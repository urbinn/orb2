/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "KeyFrameDatabase.h"
#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>

using namespace std;

namespace ORB_SLAM2
{

/**
 Initializer of the KeyFrameDatabase

 @param voc - The ORBVocabulary of the type TemplatedVocabulary which have a key of TDescriptor and value of FORB.
 @return This returns the initialized KeyFrameDatabase file resized to the size of the orb vocabulary.
 */
KeyFrameDatabase::KeyFrameDatabase(ORBVocabulary *voc) : mpVoc(voc)
{
    mvInvertedFile.resize(voc->size());
}

/**
 The keyframe is added to the inverted file for every descriptor of the keyframe.

 @param pKF - The point key frame
 */
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

/**
 This method erases the elements in the inverted file for the entry by iterating over every descriptor of the keyframe.

 @param pKF - The point keyframe
 */
void KeyFrameDatabase::erase(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

        for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
        {
            if (pKF == *lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}


/**
 This method clears the inverted file and resizes the inverted file to the associated vocabulary file.
 */
void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

/**
 This method first searches all the keyframe that share a word with the current keyframers. It also discards keyframe which are connected to the query keyframe.
 Then it compares only the keyframe which share enough words. It also computes the similarity score and retains the matches whose score is higher than the minimal score.
 When the comparison is done the score is accumulated by covisibility. Finally it returns all the keyframes with a score higher than 0.75 * best score.

 @param pKF - The given point keyframe
 @param minScore - The lowest score to a connected keyframe in the covisiblity graph.
 @return Returns a list of keyframes that are used to detect loop closure.
 */
vector<KeyFrame *> KeyFrameDatabase::DetectLoopCandidates(KeyFrame *pKF, float minScore)
{
    set<KeyFrame *> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame *> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;
                if (pKFi->mnLoopQuery != pKF->mnId)
                {
                    pKFi->mnLoopWords = 0;
                    if (!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mnLoopQuery = pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if (lKFsSharingWords.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        if ((*lit)->mnLoopWords > maxCommonWords)
            maxCommonWords = (*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    int nscores = 0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;

        if (pKFi->mnLoopWords > minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if (si >= minScore)
                lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
    {
        KeyFrame *pKFi = it->second;
        vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame *pBestKF = pKFi;
        for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
        {
            KeyFrame *pKF2 = *vit;
            if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords)
            {
                accScore += pKF2->mLoopScore;
                if (pKF2->mLoopScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;

    set<KeyFrame *> spAlreadyAddedKF;
    vector<KeyFrame *> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
    {
        if (it->first > minScoreToRetain)
        {
            KeyFrame *pKFi = it->second;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpLoopCandidates;
}

/**
 This method searched all the keyframes that share a word with the current frame. Then it only compares those keyframes that share enough words. After the comparison
 it computes the similarity score. When the computation is done, the score is accumulated by covisiblity. Finally it returns all those keyframe with a score higher
 than 0.75 * the best score. This method is used when the tracking is lost.

 @param F - The current frame.
 @return It returns a list of keyframes which are candidates for relocalization.
 */
vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    list<KeyFrame *> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;
                if (pKFi->mnRelocQuery != F->mnId)
                {
                    pKFi->mnRelocWords = 0;
                    pKFi->mnRelocQuery = F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if (lKFsSharingWords.empty())
        return vector<KeyFrame *>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        if ((*lit)->mnRelocWords > maxCommonWords)
            maxCommonWords = (*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    list<pair<float, KeyFrame *>> lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;

        if (pKFi->mnRelocWords > minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
            pKFi->mRelocScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
    {
        KeyFrame *pKFi = it->second;
        vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame *pBestKF = pKFi;
        for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
        {
            KeyFrame *pKF2 = *vit;
            if (pKF2->mnRelocQuery != F->mnId)
                continue;

            accScore += pKF2->mRelocScore;
            if (pKF2->mRelocScore > bestScore)
            {
                pBestKF = pKF2;
                bestScore = pKF2->mRelocScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;
    set<KeyFrame *> spAlreadyAddedKF;
    vector<KeyFrame *> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
    {
        const float &si = it->first;
        if (si > minScoreToRetain)
        {
            KeyFrame *pKFi = it->second;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

template <class Archive>
void KeyFrameDatabase::serialize(Archive &ar, const unsigned int version)
{
    // don't save associated vocabulary, KFDB restore by created explicitly from a new ORBvocabulary instance
    // inverted file
    ar &mvInvertedFile;
    // don't save mutex
}
template void KeyFrameDatabase::serialize(boost::archive::binary_iarchive &, const unsigned int);
template void KeyFrameDatabase::serialize(boost::archive::binary_oarchive &, const unsigned int);

} //namespace ORB_SLAM
