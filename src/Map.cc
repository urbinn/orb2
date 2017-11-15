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

#include "Map.h"

#include<mutex>


#define TEST_DATA 0xdeadbeef

namespace ORB_SLAM2
{

/**
Constructor for Map, initializes mnMaxKFid && mnBigChangeIdx with 0
*/
Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}



//template

template<class Archive>
    void Map::save(Archive & ar, const unsigned int version) const
    {
        unsigned int test_data = TEST_DATA;
        int nItems = mspMapPoints.size();
        ar & nItems;
        cout << "{INFO}mspMapPoints size = " << nItems << endl;
        std::for_each(mspMapPoints.begin(), mspMapPoints.end(), [&ar](MapPoint* pMapPoint) {
            ar & *pMapPoint;
        });

        nItems = mspKeyFrames.size();
        cout << "{INFO}mspKeyFrames size = " << nItems << endl;
        ar & nItems;
        std::for_each(mspKeyFrames.begin(), mspKeyFrames.end(), [&ar](KeyFrame* pKeyFrame) {
            ar & *pKeyFrame;
        });

        nItems = mvpKeyFrameOrigins.size();
        cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << endl;
        ar & nItems;
        std::for_each(mvpKeyFrameOrigins.begin(), mvpKeyFrameOrigins.end(), [&ar](KeyFrame* pKeyFrameOrigin) {
            ar & *pKeyFrameOrigin;
        });
        // Pertaining to map drawing
        //nItems = mvpReferenceMapPoints.size();
        //cout << "$${INFO}mvpReferenceMapPoints size = %d " << nItems << endl;
        //ar & nItems;
        //std::for_each(mvpReferenceMapPoints.begin(), mvpReferenceMapPoints.end(), [&ar](MapPoint* pMapPointReference) {
        //    ar & *pMapPointReference;
        //});
        ar & const_cast<long unsigned int &> (mnMaxKFid);

        ar & test_data;
    }

    template<class Archive>
    void Map::load(Archive & ar, const unsigned int version)
    {
        unsigned int test_data;

        int nItems;
        ar & nItems;
        cout << "{INFO}mspMapPoints size = " << nItems << endl;

        for (int i = 0; i < nItems; ++i) {

            MapPoint* pMapPoint = new MapPoint();
            ar & *pMapPoint;
            mspMapPoints.insert(pMapPoint);
        }

        ar & nItems;
        cout << "{INFO}mspKeyFrames size = " << nItems << endl;

        for (int i = 0; i < nItems; ++i) {

            KeyFrame* pKeyFrame = new KeyFrame;
            ar & *pKeyFrame;
            mspKeyFrames.insert(pKeyFrame);
        }


        ar & nItems;
        cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << endl;

        for (int i = 0; i < nItems; ++i) {

            KeyFrame* pKeyFrame = new KeyFrame;
            ar & *pKeyFrame;
			/* TODO : VerifyHere*/
            mvpKeyFrameOrigins.push_back(*mspKeyFrames.begin());
        }

        ar & const_cast<long unsigned int &> (mnMaxKFid);

        ar & test_data;
        if (test_data == TEST_DATA)
            cout <<">>Map Loading Validated as True" << endl;
        else
            cout <<"ERROR Map Loading Validated as False: Got -" << test_data << " :( Check Load Save sequence" << endl;

    }


// Explicit template instantiation
template void Map::save<boost::archive::binary_oarchive>(
	boost::archive::binary_oarchive &,
	const unsigned int) const;
template void Map::save<boost::archive::binary_iarchive>(
	boost::archive::binary_iarchive &,
	const unsigned int) const;
template void Map::load<boost::archive::binary_oarchive>(
	boost::archive::binary_oarchive &,
	const unsigned int);
template void Map::load<boost::archive::binary_iarchive>(
	boost::archive::binary_iarchive &,
	const unsigned int);







/**
This method adds KeyPoint pKF to mspKeyFrames

@param KeyFrame pKF
*/
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

/**
This method adds MapPoint pMP to mspMapPoints

@param MapPoint pMP
*/
void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

/**
This method removes the pointer of  MapPoint pMP in mspMapPoints

@param MapPoint pMP
*/
void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/**
This method removes the pointer of  KeyFrame pMP in mspKeyFrames

@param KeyFrame pKF
*/
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/**
This method sets the mvpReferenceMapPoints as the supplied a vector of MapPoints

@param vector<MapPoint>
*/
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

/**
Raises the mnBigChangeIdx (Big Change Index) by 1
This index gets updated at a big change in the map (loop closure, global BA)
*/
void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

/**
Retruns the mnBigChangeIdx  (Big Change Index)
This index counts every big change in the map (loop closure, global BA)

@return int mnBigChangeIdx
*/
int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

/**
Retruns a vector containing all the KeyFrames

@return vector<KeyFrame*>
*/
vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

/**
Retruns a vector containing all the MapPoints

@return vector<MapPoint*>
*/
vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

/**
Retruns the amount of MapPoints contained in the map

@return long unsigned int
*/
long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

/**
Retruns the amount of keyframes contained in the map

@return long unsigned int
*/
long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

/**
Retruns all the referenced MapPoints contained in the map

@return vector<MapPoint*>
*/
vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

/**
Return the highest id of a KeyFrame from the map

@return mnMaxKFid
*/
long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

/**
Reinitializes the map, this removes all existing data.
*/
void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

/**
Archives the map, this writes all existing data to Archive `ar`.

@param Archive ar
@param A const unsigned int version

template<class Archive>
void Map::serialize(Archive &ar, const unsigned int version)
{
    // don't save mutex
    ar & mspMapPoints;
    ar & mvpKeyFrameOrigins;
    ar & mspKeyFrames;
    ar & mvpReferenceMapPoints;
    ar & mnMaxKFid & mnBigChangeIdx;
}
template void Map::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void Map::serialize(boost::archive::binary_oarchive&, const unsigned int);

*/
} //namespace ORB_SLAM
