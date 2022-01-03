#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "blob.h"

using namespace std;
using namespace Eigen;

static const int NUM_CHANNELS = 1;

struct QuadTreeNode {

    inline bool isLeaf(int index) const {
        return children[index] == 0;
    }

    inline bool isLeaf() const {
        return isLeaf(0) || isLeaf(1) || isLeaf(2) || isLeaf(3);
    }

    int computeDepth(const vector<QuadTreeNode>& nodes) const {
        int maxDepth = 0;
        for (int i = 0; i < 4; ++i) {
            if (!isLeaf(i)) {
                maxDepth = max(maxDepth, nodes[children[i]].computeDepth(nodes) + 1);
            }
        }
        return maxDepth;
    }

    float computeMax(const vector<QuadTreeNode>& nodes) const {
        float maximum = 0;
        for (int i = 0; i < 4; ++i) {
            if (!isLeaf(i)) {
                maximum = max(maximum, nodes[children[i]].computeMax(nodes));
            } else {
                maximum = max(maximum, data[0][i]);
            }
        }
        return 4 * maximum;
    }

    int getChildIndex(Vector2f& p) const {
        if (p.x() < 0.5f) {
            p.x() *= 2;
            if (p.y() < 0.5f) {
                p.y() *= 2;
                return 0;
            } else {
                p.y() = (p.y() - 0.5f) * 2;
                return 1;
            }
        } else {
            p.x() = (p.x() - 0.5f) * 2;
            if (p.y() < 0.5f) {
                p.y() *= 2;
                return 2;
            } else {
                p.y() = (p.y() - 0.5f) * 2;
                return 3;
            }
        }
    }

    float eval(int index, Vector2f& p, const vector<QuadTreeNode>& nodes) const {
        const int c = getChildIndex(p);
        if (isLeaf(c)) {
            return data[index][c];
        } else {
            return 4 * nodes[children[c]].eval(index, p, nodes);
        }
    }

    array<array<float, 4>, NUM_CHANNELS> data;
    array<uint16_t, 4> children;

    Vector2f getChildCoords(int index) {
        switch (index) {
            case 0: return Vector2f(0.25f, 0.25f);
            case 1: return Vector2f(0.25f, 0.75f);
            case 2: return Vector2f(0.75f, 0.25f);
            case 3: return Vector2f(0.75f, 0.75f);
            default: return Vector2f(0, 0);
        }
    }

    Vector2f coords;
};

class DTree {
public:
    bool read(BlobReader& blob) {

        blob
            >> mPos.x() >> mPos.y() >> mPos.z()
            >> mSize.x() >> mSize.y() >> mSize.z()
            >> mMean >> numSamples >> numNodes;

        if (!blob.isValid()) {
            return false;
        }

        cout << numNodes << endl;

        mNumSamples = (size_t) numSamples;

        if (!isfinite(mMean)) {
            cerr << "INVALID MEAN: " << mMean << endl;
        }

        mPos += mSize / 2;

        mNodes.resize(numNodes);
        for (auto& node : mNodes) {
            for (int j = 0; j < 4; ++j) {
                for (int k = 0; k < NUM_CHANNELS; ++k) {
                    blob >> node.data[k][j];
                    if (!isfinite(node.data[k][j])) {
                        cerr << "INVALID NODE: " << node.data[k][j] << endl;
                    }
                }
                blob >> node.children[j];
            }
        }

        mDepth = computeDepth();
        mMax = computeMax();

        return true;
    }

    void write(BlobWriter& blob) {

        mPos -= mSize / 2;

        blob
            << mPos.x() << mPos.y() << mPos.z()
            << mSize.x() << mSize.y() << mSize.z()
            << mMean << numSamples << numNodes;

        for (auto& node : mNodes) {
            for (int j = 0; j < 4; ++j) {
                for (int k = 0; k < NUM_CHANNELS; ++k) {
                    blob << node.data[k][j];
                }
                blob << node.children[j];
            }
        }
    }

    float eval(int index, Vector2f p) const {
        if (mNumSamples == 0) {
            return 0;
        }
        const float factor = 1 / (float) (M_PI * (double) mNumSamples);
        return factor * mNodes[0].eval(index, p, mNodes);
    }

    void computeCoords(int index, Vector2f coords) {
        QuadTreeNode currentNode = mNodes[index];
        currentNode.coords = coords;

        if(currentNode.isLeaf()) return;

        for(int i = 0; i < currentNode.children.size(); i++) {
            computeCoords(currentNode.children[i], currentNode.getChildCoords(i) * coords);
        }
    }

    vector<QuadTreeNode> mNodes;

private:
    uint64_t numNodes;
    uint64_t numSamples;
    size_t mNumSamples;

    Vector3f mPos;
    Vector3f mSize;

    float mMean;
    int mDepth;
    float mMax;

    int computeDepth() const {
        return mNodes[0].computeDepth(mNodes) + 1;
    }

    float computeMax() const {
        if (mNumSamples == 0) {
            return 0;
        }
        const float factor = 1 / (float) (4 * M_PI * (double) mNumSamples);
        return factor * mNodes[0].computeMax(mNodes);
    }
};

class SDTree {
public:

    SDTree() {}
    SDTree(vector<DTree>& dTrees, Matrix4f& camera) : mDTrees(dTrees), mCamera(camera) {}

    void loadSDTree(const string& filename) {

        BlobReader reader(filename);

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                reader >> mCamera(i, j);
            }
        }

        while (true) {
            DTree dTree;
            if (!dTree.read(reader)) {
                break;
            }

            mDTrees.push_back(dTree);
        }

        cout << "Loaded SD-Tree with " << mDTrees.size() << " D-Trees." << endl;

        setDTree(0);
    }

    void dumpSDTree(const string& filename) {

        BlobWriter writer(filename);

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                writer << mCamera(i, j);
            }
        }

        for (auto &dTree: mDTrees) {
            dTree.write(writer);
        }

        cout << "Saved SD-Tree with " << mDTrees.size() << " D-Trees." << endl;
    }

    void setDTree(size_t i) {
        currentDTreeIndex = i;
    }

    vector<DTree> mDTrees;
    Matrix4f mCamera;

private:

    size_t currentDTreeIndex = -1;
};


