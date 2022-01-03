#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>

#include "trees.hpp"

using namespace std;
using namespace cv;

class ppnlm_denoiser {
private:

    vector<DTree> dTrees;

    void sortNodes(DTree& dTree, int index, int level, vector<vector<QuadTreeNode>>& nodesByLevel,
                   Vector2i coords, vector<vector<Vector2i>>& coordsByLevel) {

        QuadTreeNode currentNode = dTree.mNodes[index];
        nodesByLevel[level].push_back(currentNode);
        coordsByLevel[level].push_back(coords);

        if(currentNode.isLeaf()) return;

        for(int i = 0; i < 4; i++) {
            Vector2i childCoords;
            if(i == 0) childCoords = Vector2i(2 * coords.x(), 2 * coords.y());
            if(i == 1) childCoords = Vector2i(2 * coords.x(), 2 * coords.y() + 1);
            if(i == 2) childCoords = Vector2i(2 * coords.x() + 1, 2 * coords.y());
            if(i == 3) childCoords = Vector2i(2 * coords.x() + 1, 2 * coords.y() + 1);
            sortNodes(dTree, currentNode.children[i], ++level, nodesByLevel, childCoords, coordsByLevel);
        }
    }

    DTree denoise(DTree& dTree) {
        vector<vector<QuadTreeNode>> nodesByLevels;
        vector<vector<Vector2i>> coordsByLevel;
        sortNodes(dTree, 0, 0, nodesByLevels, Vector2i(0, 0), coordsByLevel);
        //first pass for neighbors and levels
        //start from bottom up
    }

public:

    SDTree denoise(SDTree& sdTree) {
        for(DTree dTree: sdTree.mDTrees) {
            dTrees.push_back(denoise(dTree));
        }
        return SDTree(dTrees, sdTree.mCamera);
    }
};