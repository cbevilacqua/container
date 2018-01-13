// Given an container of points, write a function which will return the pair of closest points
// Feel free to dictate the nature of the container
// Work through an implementation of a general solution which can support any definition of a point and an arbitrary coordinate frame and definition of “closest”

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

using namespace std;

// very general Container implementation

template <class PointType, class DistanceType>
class Container {
   public:
       Container(DistanceType (*distanceFunction)(const PointType&, const PointType&));
       
       void addPoint(PointType point);
       pair<PointType*, PointType*> findClosestPair();
   private:
       DistanceType (*mDistanceFunction)(const PointType&, const PointType&);
       vector<PointType> mPoints;
};

template <class PointType, class DistanceType>
Container<PointType, DistanceType>::Container(DistanceType (*distanceFunction)(const PointType&, const PointType&))
{
    mDistanceFunction = distanceFunction;
}

template <class PointType, class DistanceType>
void Container<PointType, DistanceType>::addPoint(PointType point)
{
   mPoints.push_back(point); 
}

// This implementation assumes that mDistanceFunction(a,b) == mDistanceFunction(b,a).
// This could be modified easily, or a template specialization could be created for PointTypes
//      for which this does not apply (nodes in a directed graph, for example).
template <class PointType, class DistanceType>
pair<PointType*, PointType*> Container<PointType, DistanceType>::findClosestPair()
{
    int numberPoints = mPoints.size();
    if (numberPoints < 2)
    {
        return pair<PointType*, PointType*>(nullptr, nullptr);
    }
    
    DistanceType closestDistanceFound = mDistanceFunction(mPoints[numberPoints-2], mPoints[numberPoints-1]);
    pair<PointType*, PointType*> closestPairFound(&mPoints[numberPoints-2], &mPoints[numberPoints-1]);
    
    for (int i = 0; i < numberPoints-2; i++)
    {
        for (int j = i+1; j < numberPoints-1; j++)
        {
            DistanceType newDistance = mDistanceFunction(mPoints[i], mPoints[j]);
            if (newDistance < closestDistanceFound)
            {
                closestDistanceFound = newDistance;
                closestPairFound.first = &mPoints[i];
                closestPairFound.second = &mPoints[j];
            }
        }
    }
    
    return closestPairFound;
}

//------------------------
// Simple use-case with 2d cartesian points

struct CartesianPoint
{
        CartesianPoint(float x, float y)
        {
            mX = x;
            mY = y;
        }
        float mX;
        float mY;
};

float cartesianDistance(const CartesianPoint& a, const CartesianPoint& b)
{
    return sqrt(pow(b.mY-a.mY, 2) + pow(b.mX-a.mX, 2));
}

// Utility function for this file, not something I would consider part of 
//     the Container class since it relies on specific definition of PointType
void printPair(pair<CartesianPoint*, CartesianPoint*> myPair)
{
    if (myPair.first)
    {
        cout << "(" << myPair.first->mX << ", " << myPair.first->mY << ")";
        cout << " and ";
        cout << "(" << myPair.second->mX << ", " << myPair.second->mY << ")";
        cout << endl;
    }
    else
        cout << "No pair exists." << endl;
}

//------------------------
// Template specialization for a use-case with a findClosestPair implementation that relies on
//    unique information of the class.
// Graph with no negatively weighted edges, with each node represented as an adjacency list,
//    i.e., the "points" in the container are pointers to the beginning of each node's adjacency list.
// Use Djikstra's algorithm n times to get all n^2 distances
// Not everything is fully implemented - just enough to show the template specialization
//    and necessary functions

template<class DistanceType>
struct adjacencyListNode{
  adjacencyListNode(int y, DistanceType edgeWeight)
  {
      mY = y; // I.e., if this adjacency list is for node x, this particular edge is (x,y)
      mEdgeWeight = edgeWeight;
  }
  int mY;
  DistanceType mEdgeWeight;
  adjacencyListNode* mNext = nullptr;
};

template <class DistanceType>
class Container<adjacencyListNode<DistanceType>, DistanceType> {
    public:
        Container(vector<vector<DistanceType>> (*allPairsShortestPathFunction)(const vector<adjacencyListNode<DistanceType>>&), bool directed, int numberPoints = 0);
        
        void addPoint();
        void addEdge(int x, int y, DistanceType edgeWeight);
        pair<int, int> findClosestPair();
    private:
        vector<vector<DistanceType>> (*mAllPairsShortestPathFunction)(const vector<adjacencyListNode<DistanceType>>&);
        bool mDirected;
        vector<adjacencyListNode<DistanceType>*> mPoints;
};

template <class DistanceType>
Container<adjacencyListNode<DistanceType>, DistanceType>::Container(vector<vector<DistanceType>> (*allPairsShortestPathFunction)(const vector<adjacencyListNode<DistanceType>>&), bool directed, int numberPoints) :
    mDirected(directed),
    mPoints(numberPoints, nullptr)
{
    mAllPairsShortestPathFunction = allPairsShortestPathFunction;
}

template <class DistanceType>
void Container<adjacencyListNode<DistanceType>, DistanceType>::addPoint()
{
    mPoints.push_back(nullptr);
}

template <class DistanceType>
void Container<adjacencyListNode<DistanceType>, DistanceType>::addEdge(int x, int y, DistanceType edgeWeight)
{
    if (x < 0 || x > mPoints.size()-1 || y < 0 || y > mPoints.size()-1 || x == y)
        return;
    
    // doesn't check for redundant edges at this time - currently the responsibility of the user,
    //   but a check could be added if necessary
    adjacencyListNode<DistanceType>* newEdge = new adjacencyListNode<DistanceType>(y, edgeWeight);
    newEdge->mNext = mPoints[x];
    mPoints[x] = newEdge;
    
    if (!mDirected)
    {
        adjacencyListNode<DistanceType>* correspondingEdge = new adjacencyListNode<DistanceType>(x, edgeWeight);
        correspondingEdge->mNext = mPoints[y];
        mPoints[y] = correspondingEdge;
    }
}

template <class DistanceType>
pair<int, int> Container<adjacencyListNode<DistanceType>, DistanceType>::findClosestPair()
{
    if (mPoints.size() < 2)
        return pair<int, int>(-1, -1);
    
    auto shortestPaths = allPairsShortestPath(mPoints);
    
    DistanceType lowestDistanceFound = std::numeric_limits<DistanceType>::max();
    pair<int, int> closestPairFound(-1, -1);
    
    for (int i = 0; i < mPoints.size(); i++)
    {
        for (int j = 0; j < mPoints.size(); j++)
        {
            if (shortestPaths[i][j] <= lowestDistanceFound)
            {
                lowestDistanceFound = shortestPaths[i][j];
                closestPairFound.first = i;
                closestPairFound.second = j;
            }
        }
    }
    
    return closestPairFound;
}

template <class DistanceType>
vector<vector<DistanceType>> allPairsShortestPath(const vector<adjacencyListNode<DistanceType>>& points)
{
    int numberPoints = points.size();
    vector<vector<DistanceType>> shortestPaths(numberPoints, vector<DistanceType>(numberPoints, std::numeric_limits<DistanceType>::max()));

    // We could conduct Djikstra's algorithm n times to fill out one row of shortestPaths at a time,
    //     or we could perform the Floyd-Warshall algorithm.
    // This is left unimplemented for now.

    return shortestPaths;
}

//------------------------

int main()
{
    Container<CartesianPoint, float> myContainer(cartesianDistance);
    
    myContainer.addPoint(CartesianPoint(2, 3));
    
    printPair(myContainer.findClosestPair());
    
    myContainer.addPoint(CartesianPoint(2, 10));
    myContainer.addPoint(CartesianPoint(2, 4));
    myContainer.addPoint(CartesianPoint(5, 10));
    
    printPair(myContainer.findClosestPair());
    
    //------------------------
    
    Container<adjacencyListNode<int>, int> myOtherContainer(allPairsShortestPath, false, 10); 
    
    return 0;
}
