/*
    void writeMats(vector<Mat>& mats) {

        cout << "writing mats" << endl;

        for(Mat mat : mats) {

            double min, max;
            minMaxIdx(mat, &min, &max);
            if(max != 0) mat = mat / max;

            Mat output;
            mat.convertTo(output, CV_8U, 255.0);
            imwrite("output/mats/mat_" + to_string(output.rows) + ".png", output);
        }
    }

    vector<Mat> createMats(vector<vector<Vec2i>>& coordsByLevel,
                           vector<vector<float>>& valsByLevel) {

        vector<Mat> mats;

        assert((coordsByLevel.size() == valsByLevel.size()));
        for(int i = 0; i < coordsByLevel.size(); i++) {

            vector<Vec2i> currentCoords = coordsByLevel[i];
            vector<float> currentVals = valsByLevel[i];
            Mat currentMat((int) pow(2, i), (int) pow(2, i), CV_32F);

            assert((currentCoords.size() == currentVals.size()));
            for(int j = 0; j < currentCoords.size(); j++) {
                currentMat.at<float>(currentCoords[j]) = currentVals[j];
            }

            mats.push_back(currentMat);
        }

        return mats;
    }
     */

