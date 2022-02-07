#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), faces_(), vt_() {
    std::ifstream in;
    in.open (filename, std::ifstream::in);
    if (in.fail()) return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f v;
            for (int i=0;i<3;i++) iss >> v.raw[i];
            verts_.push_back(v);
        } else if (!line.compare(0, 2, "f ")) {
            std::vector<int> f;
            int itrash, idx, idx2;
            iss >> trash;
            while (iss >> idx >> trash >> idx2  >> trash >> itrash) {
                idx--; // in wavefront obj all indices start at 1, not zero
                idx2--;
                f.push_back(idx);
                f.push_back(idx2);
            }
            faces_.push_back(f);
            //std::cout << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << " " << f[4] << " " << f[5] << "\n";
        } else if (!line.compare(0, 3, "vt ")) {
            iss >> trash;
            iss >> trash;
            Vec3f vt;
            for(int i = 0 ; i < 3 ; i++) {
                iss >> vt.raw[i];
            }
            vt_.push_back(vt);
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() {
    return (int)verts_.size();
}

int Model::nfaces() {
    return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
    return faces_[idx];
}

Vec3f Model::vert(int i) {
    return verts_[i];
}

Vec3f Model::vt(int i) {
    return vt_[i];
}

int Model::nvt() {
    return (int)vt_.size();
}

