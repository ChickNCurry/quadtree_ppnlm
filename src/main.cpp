#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <unordered_map>

using namespace std;
using namespace Eigen;
using namespace cv;

static const int NUM_CHANNELS = 1;

class BlobReader {
public:
    BlobReader(const string& filename) : f(filename, ios::in | ios::binary) {}

    template <typename Type>
    typename enable_if<is_standard_layout<Type>::value, BlobReader&>::type
    operator >> (Type& Element) {
        Read(&Element, 1);
        return *this;
    }

    template <typename T>
    void Read(T* Dest, size_t Size) {
        f.read(reinterpret_cast<char*>(Dest), Size * sizeof(T));
    }

    bool isValid() const {
        return (bool)(f);
    }

private:
    ifstream f;
};

class BlobWriter {
public:
    BlobWriter(const std::string& filename): f(filename, ios::out | ios::binary) {}

    template <typename Type>
    typename std::enable_if<std::is_standard_layout<Type>::value, BlobWriter&>::type
    operator << (Type& Element) {
        Write(&Element, 1);
        return *this;
    }

    template <typename T>
    void Write(T* Src, size_t Size) {
        f.write(reinterpret_cast<char*>(Src), Size * sizeof(T));
    }

private:
    ofstream f;
};

struct QuadTreeNode {

    inline bool isLeaf(int index) const {
        return children[index] == 0;
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

    int computeDepth() const {
        return mNodes[0].computeDepth(mNodes) + 1;
    }

    vector<QuadTreeNode> mNodes;
    int mDepth;
    float mMax;
    float radiance;

private:
    uint64_t numNodes;
    uint64_t numSamples;
    size_t mNumSamples;

    Vector3f mPos;
    Vector3f mSize;

    float mMean;

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

    vector<DTree> mDTrees;
    Matrix4f mCamera;
};

struct NodeData {
    int index;
    int parentIndex;
    int whichChild; // 0 - 3
    float value;
    Vec2i coords;
    float rel;
    NodeData(int i, int p, int w, float v, const Vec2i& c, float r) :
    index(i), parentIndex(p), whichChild(w), value(v), coords(c), rel(r) {}
};

class dTreeDenoiser {
private:

    DTree& dTree;

    const int max_level;
    const float smoothing = 1;

    const float rel_max = 1;
    const float upper_bound = rel_max - (rel_max / 3);
    const float lower_bound = rel_max / 3;
    const float divider = rel_max / 3;

    vector<vector<shared_ptr<NodeData>>> downfuse_levels;
    vector<unordered_map<string, shared_ptr<NodeData>>> maps;
    vector<vector<shared_ptr<NodeData>>> upfuse_levels;

    void upfuse(int level) {
        cout << "upfuse level: " << level << endl;
        if(level > max_level) return;

        for(auto& node: upfuse_levels[level]) {

        }
    }

    vector<shared_ptr<NodeData>> getPatch(int level, shared_ptr<NodeData>& node) {
        vector<shared_ptr<NodeData>> patch;
        patch.push_back(node);
        patch.push_back(findHashedNode(Vec2i(node->coords[0] + 1, node->coords[1]), level));
        patch.push_back(findHashedNode(Vec2i(node->coords[0], node->coords[1] + 1), level));
        patch.push_back(findHashedNode(Vec2i(node->coords[0] + 1, node->coords[1] + 1), level));
    }

    float calculateDelta(vector<shared_ptr<NodeData>>& local_patch, vector<shared_ptr<NodeData>>& other_patch) const {
        float sum = 0;

        for(int i = 0; i < local_patch.size(); i++) {
            float diff = (1 / smoothing) * (local_patch[i]->value - other_patch[i]->value);
            sum += (float) pow(diff, 2);
        }

        return exp(-sum);
    }

    float downfuseNode(int level, shared_ptr<NodeData>& origin_node, float& weight_sum, float& weight_sqrd_sum) {

        float value_sum = 0;

        vector<shared_ptr<NodeData>> local_patch = getPatch(level, origin_node);

        for(auto& node: downfuse_levels[level]) {

            if(!(node->coords[0] % 2 == 0 && node->coords[1] % 2 == 0)) continue;

            vector<shared_ptr<NodeData>> other_patch = getPatch(level, node);
            float delta = calculateDelta(local_patch, other_patch);

            float weight = node->rel * delta;

            weight_sum += weight;
            weight_sqrd_sum += (float) pow(weight, 2);

            value_sum += weight * node->value;
        }

        return value_sum;
    }

    void downfuse(int level) {
        cout << "downfuse level: " << level << endl;
        if(level < 0) return;

        for(auto& node: downfuse_levels[level]) {

            if(!(node->coords[0] % 2 == 0 && node->coords[1] % 2 == 0)) continue;

            float weight_sum = 0;
            float weight_sqrd_sum = 0;

            float val = downfuseNode(level, node, weight_sum, weight_sqrd_sum);

            Vec2i parentCoords;
            switch (node->whichChild) {
                case 0: parentCoords = Vec2i(node->coords / 2);
                    break;
                case 1: parentCoords = Vec2i(node->coords[0] / 2, (node->coords[1] - 1) / 2);
                    break;
                case 2: parentCoords = Vec2i((node->coords[0] - 1) / 2, node->coords[1] / 2);
                    break;
                case 3: parentCoords = Vec2i((node->coords[0] - 1) / 2, (node->coords[1] - 1) / 2);
                    break;
            }

            if(weight_sum != 0) {
                float normalized_val = val / weight_sum;
                assert((normalized_val >= 0 && normalized_val <= 1));

                shared_ptr<NodeData> parent = findHashedNode(parentCoords, level - 1);
                parent->value = normalized_val;
            } else {
                shared_ptr<NodeData> parent = findHashedNode(parentCoords, level - 1);
                parent->value = 0;
            }

            if(weight_sqrd_sum != 0) {
                float rel_output = ((float) pow(weight_sum, 2)) / weight_sqrd_sum;
                assert((rel_output >= 1 && rel_output <= rel_max));

                shared_ptr<NodeData> parent = findHashedNode(parentCoords, level - 1);
                parent->rel = rel_output;
            } else {
                shared_ptr<NodeData> parent = findHashedNode(parentCoords, level - 1);
                parent->rel = 1;
            }
        }

        downfuse(--level);
    }

    void clampRels() {
        for(auto& level: downfuse_levels) {
            for(auto& node: level) {

                auto clamped_val = (float) fmin(fmax(0, node->rel - lower_bound), rel_max - upper_bound);
                float normalized_val = clamped_val / divider;

                assert((normalized_val >= 0 && normalized_val <= 1));
                node->rel = normalized_val;
            }
        }
    }

    void ppnlm() {
        downfuse(max_level);
        downfuse_levels = upfuse_levels;
        clampRels();
        upfuse(0);
    }

    /*
    void reconstructDTree() {
        for(const auto& level: nodesByLevel) {
            for(const auto& node: level) {
                //cout << dTree.mNodes[node.parentIndex].data[0][node.whichChild] << endl;
                dTree.mNodes[node.parentIndex].data[0][node.whichChild] = node.value;
                //cout << dTree.mNodes[node.parentIndex].data[0][node.whichChild] << endl;
            }
        }
    }
     */

    void hashNodesByLevel() {
        assert((downfuse_levels.size() == maps.size()));
        for(int i = 0; i < downfuse_levels.size(); i++) {
            for(auto& node: downfuse_levels[i]) {
                maps[i].insert(make_pair(vecToString(node->coords), node));
            }
        }
    }

    void sortNodesByLevel(int level, shared_ptr<NodeData>& parent) {

        QuadTreeNode& currentNode = dTree.mNodes[parent->index];

        for(int i = 0; i < 4; i++) {

            Vec2i childCoords;
            switch (i) {
                case 0: childCoords = Vec2i(2 * parent->coords);
                    break;
                case 1: childCoords = Vec2i(2 * parent->coords[0], 2 * parent->coords[1] + 1);
                    break;
                case 2: childCoords = Vec2i(2 * parent->coords[0] + 1, 2 * parent->coords[1]);
                    break;
                case 3: childCoords = Vec2i(2 * parent->coords[0] + 1, 2 * parent->coords[1] + 1);
                    break;
            }

            shared_ptr<NodeData> child = make_shared<NodeData>(
                    NodeData(currentNode.children[i], parent->index, i,
                             currentNode.data[0][i], childCoords, rel_max));

            downfuse_levels[level].push_back(child);

            if(currentNode.isLeaf(i)) continue;

            sortNodesByLevel(level + 1, child);
        }
    }

    static string vecToString(Vec2i vec) {
        return to_string(vec[0]) + "_" + to_string(vec[1]);
    }

    shared_ptr<NodeData> findHashedNode(const Vec2i& coords, int level) {
        auto got = maps[level].find(vecToString(coords));
        if (got == maps[level].end()) throw invalid_argument("node not found");
        else return got->second;
    }

public:

    explicit dTreeDenoiser(DTree& dTree): dTree(dTree), downfuse_levels(dTree.mDepth),
                                            maps(dTree.mDepth), max_level(dTree.mDepth - 1) {}

    void denoiseDTree() {
        shared_ptr<NodeData> root = make_shared<NodeData>(
                NodeData(0, -1, -1, 0, Vec2i(0, 0), rel_max));

        sortNodesByLevel(0, root);
        hashNodesByLevel();
        ppnlm();
    }
};

SDTree denoiseSDTree(SDTree sdTree) {
    for(auto& dTree: sdTree.mDTrees) {
        dTreeDenoiser denoiser(dTree);
        denoiser.denoiseDTree();
    }
    return sdTree;
}

int main() {
    string input_path = "/home/maxb/Desktop/tree.sdt";
    string output_path = "/home/maxb/Desktop/out.sdt";
    SDTree sdTree;
    sdTree.loadSDTree(input_path);
    SDTree output = denoiseSDTree(sdTree);
    output.dumpSDTree(output_path);
    return 0;
}
