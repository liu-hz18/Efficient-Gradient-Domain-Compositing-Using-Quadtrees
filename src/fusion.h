#ifndef __FUSION_H__
#define __FUSION_H__

#include <cstring>
#include <cstdio>
#include <string>
#include <iostream>
#include <cmath>
#include <cassert>
#include <map>
#include <unordered_map>
#include <vector>
#include <utility>
#include <iomanip>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/src/IterativeLinearSolvers/ConjugateGradient.h>
#include "image.h"
#include "quadtree.h"

using namespace std;

class ImageFusion {
    bool in_mask(int x, int y) {
        if (x < xoffset || x >= xoffset + mask->h) return false;
        if (y < yoffset || y >= yoffset + mask->w) return false;
        return (mask->get_pixel(x-xoffset, y-yoffset)[0] > 127);
    }

    Eigen::SparseMatrix<double>* AtA;
    Eigen::SparseVector<double>* AtB[3];
    vector<vector<pair<int, double> > > interp;
    QuadTree* root;
    map<pair<int, int>, int> keypoints;

public:
    int xoffset, yoffset; // H, W
    int width, height;
    Image *source, *target, *mask;
    Image *copy_img, *full_img, *qt_img, *full_delta_img, *qt_delta_img;
    ImageFusion() = delete;

    ImageFusion(Image* _source, Image* _target, Image* _mask, int xoffset, int yoffset) {
        this->source = _source;
        this->target = _target;
        this->mask = _mask;
        this->xoffset = xoffset;
        this->yoffset = yoffset;
        // init width and height
        this->width = std::max(target->w, source->w + yoffset);
        this->height = std::max(target->h, source->h + xoffset);
        this->copy_img = new Image(width, height, 3);
        this->full_img = new Image(width, height, 3);
        this->qt_img = new Image(width, height, 3);
        this->full_delta_img = new Image(width, height, 3);
        this->qt_delta_img = new Image(width, height, 3);
        this->AtA = nullptr;
        for (int c = 0; c < 3; ++c) this->AtB[c] = nullptr;
        cout << "final image  width: " << width << " height: " << height << endl << endl;
    }

    ~ImageFusion() {
        if (AtA) delete AtA;
        for (int c = 0; c < 3; ++c) {
            if (AtB[c]) delete AtB[c];
        }
    }

    void apply_simple_copy () {
        cout << "simple copy..." << endl;
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                if (in_mask(i, j)) {
                    for (int c = 0; c < 3; ++c) {
                        copy_img->get_pixel(i, j)[c] = source->get_pixel(i-xoffset, j-yoffset)[c];
                    }
                } else {
                    for (int c = 0; c < 3; ++c) {
                        copy_img->get_pixel(i, j)[c] = target->get_pixel(i, j)[c];
                    }
                }
            }
        }
    }

    void save_copy_image(const string& dir) const {
        copy_img->save(dir + "/copy.png");
    }

    void setup_linear_system(const vector<vector<pair<int, double> > >& A, const vector<double> B[3], int size) {
        map<pair<int, int>, double> M;
        for (auto& line: A) {
            for (auto i: line) {
                for (auto j: line) {
                    M[make_pair(i.first, j.first)] += i.second * j.second;
                }
            }
        }
        vector<Eigen::Triplet<double> > triplets;
        for (auto& it: M){
            triplets.push_back({it.first.first, it.first.second, it.second});
        }
        AtA = new Eigen::SparseMatrix<double>(size, size);
        AtA->setFromTriplets(triplets.begin(), triplets.end());
        AtA->makeCompressed();
        for (int c = 0; c < 3; ++c) {
            AtB[c] = new Eigen::SparseVector<double>(size);
            AtB[c]->setZero();
            for (int i = 0; i < int(A.size()); ++i) {
                for (auto &it: A[i]) {
                    AtB[c]->coeffRef(it.first) += it.second * B[c][i];
                }
            }
        }
    }

    void build_full_equation() {
        cout << "build full equation..." << endl;
        vector<vector<pair<int, double> > > A;
        vector<double> B[3];
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                auto update_eq = [&](int ti, int tj) {
                    if (ti >= 0 && tj >= 0) {
                        // set A matrix
                        vector<pair<int, double> > line = {make_pair(i*width+j, 1.0), make_pair(ti*width+tj, -1.0)};
                        A.emplace_back(line);
                        // set B vector
                        if (in_mask(i, j) != in_mask(ti, tj)) {
                            for (int c = 0; c < 3; ++c) {
                                int t0 = copy_img->get_color(i, j, c) - copy_img->get_color(ti, tj, c);
                                int t1 = target->get_color(i, j, c) - target->get_color(ti, tj, c);
                                B[c].push_back(t1 - t0);
                            }
                        } else {
                            for (int c = 0; c < 3; ++c) B[c].push_back(0.0);
                        }
                    }
                };
                update_eq(i, j-1);
                update_eq(i-1, j);
            }
        }
        cout << "full problem scale: " << width*height << endl;
        cout << "full memory used: " << width*height*8 / 1024.0 / 1024.0 << " MB" << endl;
        setup_linear_system(A, B, width*height);
    }

    void apply_gradient_domain_full () {
        build_full_equation();
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver;
        solver.compute(*AtA);
        for (int c = 0; c < 3; ++c) {
            int size = height*width;
            vector<double> x(size, 0.0);
            vector<double> delta(width*height, 0.0);
            Eigen::SparseVector<double> ans = solver.solve(*AtB[c]);
            for(Eigen::SparseVector<double>::InnerIterator it(ans); it; ++it)
			    x[it.index()] = it.value();
            double mean = 0.0, max = -256, min = 256;
            for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
                    double val = x[i*width+j];
                    mean += val;
                    max = std::max(max, val);
                    min = std::min(min, val);
                    delta[i*width+j] = val;
                }
            }
            mean /= (height*width);
            for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
                    double d = delta[i*width+j];
                    int val = std::round(copy_img->get_color(i, j, c) + d - mean);
                    // int val = std::round(copy_img->get_color(i, j, c) + d);
                    full_img->get_pixel(i, j)[c] = std::max(std::min(val, 255), 0);
                    // reduce to [0, 255] for stability and positive constraints
                    full_delta_img->get_pixel(i, j)[c] = 255 * (d - min) / (max - min);
                }
            }
        }
    }

    void save_full_fusion_image(const string& dir) const {
        full_img->save(dir + "/full.png");
        full_delta_img->save(dir + "/full_delta.png");
    }

    void build_quadtree() {
        cout << "building QuadTree..." << endl;
        int range = 1, seam_count = 0;
        for (int t = std::max(width, height); range < t; range <<= 1);
        root = new QuadTree(0, range, 0, range);
        int direction[4][2] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };
        for (int i = 0; i < width; i++) {
            root->split(height-1, i, 1);
        }
        for (int i = 0; i < height; ++i) {
            root->split(i, width-1, 1);
        }
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                for (int k = 0; k < 4; ++k) {
                    int ti = i + direction[k][0];
                    int tj = j + direction[k][1];
                    if (ti >= 0 && ti < height && tj >= 0 && tj < width) {
                        if (in_mask(i, j) != in_mask(ti, tj)) {
                            root->split(i, j, 1);
                            ++seam_count;
                            break;
                        }
                    }
                }
            }
        }
        cout << "find seam points: " << seam_count << endl;
        int keypoints_count = 0;
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                if (root->on_seam(i, j))
                    keypoints[make_pair(i, j)] = keypoints_count++;
            }
        }
        // cout << "find key points: " << keypoints_count << endl;
    }

    void build_quadtree_equation() {
        cout << "building QuadTree Equation..." << endl;
        // build interpolation matrix S
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                vector<pair<int, double> > line;
                auto it = keypoints.find({i, j});
                if (it != keypoints.end()) line.push_back({ it->second, 1.0 });
                else {
                    // interpolation node (i, j)
                    unordered_map<int, double> weight;
                    QuadTree* node = root->find_rightstrict(i, j);
                    double area = node->range * node->range;
                    int X[4] = {
                        node->xl, node->xl, node->xr, node->xr
                    };
                    int Y[4] = {
                        node->yl, node->yr, node->yl, node->yr
                    };
                    double W[4] = {
                        (node->xr - i) * (node->yr - j) / area,
                        (node->xr - i) * (j - node->yl) / area,
                        (i - node->xl) * (node->yr - j) / area,
                        (i - node->xl) * (j - node->yl) / area
                    };
                    for (int t = 0; t < 4; ++t)  {
                        if (W[t] < 1e-5) continue;
                        auto it = keypoints.find({X[t], Y[t]});
                        if (it != keypoints.end()) {
                            weight[it->second] += W[t];
                        } else {
                            auto interpolation = [&](QuadTree* n) {
                                if (X[t] == n->xl || X[t] == n->xr) {
                                    auto it_l = keypoints.find({X[t], n->yl});
                                    auto it_r = keypoints.find({X[t], n->yr});
                                    if (it_l != keypoints.end() && it_r != keypoints.end()) {
                                        weight[it_r->second] += W[t] * (Y[t] - n->yl) / (n->yr - n->yl);
                                        weight[it_l->second] += W[t] * (n->yr - Y[t]) / (n->yr - n->yl);
                                    }
                                }
                                if (Y[t] == n->yl || Y[t] == n->yr) {
                                    auto it_l = keypoints.find({n->xl, Y[t]});
                                    auto it_r = keypoints.find({n->xr, Y[t]});
                                    if (it_l != keypoints.end() && it_r != keypoints.end()) {
                                        weight[it_r->second] += W[t] * (X[t] - n->xl) / (n->xr - n->xl);
                                        weight[it_l->second] += W[t] * (n->xr - X[t]) / (n->xr - n->xl);
                                    }
                                }
                            };
                            interpolation(root->find_rightstrict(X[t], Y[t]));
                            interpolation(root->find_leftstrict(X[t], Y[t]));
                        }
                    }
                    for (auto it: weight) {
                        line.push_back(it);
                    }
                }
                interp.emplace_back(line);
            }
        }
        // build A, S and B
        vector<vector<pair<int, double> > > A;
        vector<double> B[3];
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                auto update_eq = [&](int ti, int tj) {
                    if (ti >= 0 && tj >= 0) {
                        // set A matrix
                        map<int, double> line;
                        for (auto mv: interp[i*width+j]) line[mv.first] += mv.second;
                        for (auto mv: interp[ti*width+tj]) line[mv.first] -= mv.second;
                        vector<pair<int, double> > tmp_line(line.begin(), line.end());
                        A.emplace_back(tmp_line);
                        // set B vector
                        if (in_mask(i, j) != in_mask(ti, tj)) {
                            for (int c = 0; c < 3; ++c) {
                                int t0 = copy_img->get_color(i, j, c) - copy_img->get_color(ti, tj, c);
                                int t1 = target->get_color(i, j, c) - target->get_color(ti, tj, c);
                                B[c].push_back(t1 - t0);
                            }
                        } else {
                            for (int c = 0; c < 3; ++c) B[c].push_back(0.0);
                        }
                    }
                };
                update_eq(i, j-1);
                update_eq(i-1, j);
            }
        }
        cout << "quadtree problem scale: " << keypoints.size() << endl;
        cout << "quadtree memory used: " << keypoints.size()*8 / 1024.0 / 1024.0 << " MB" << endl;
        setup_linear_system(A, B, keypoints.size());
    }

    void apply_gradient_domain_quadtree() {
        build_quadtree();
        build_quadtree_equation();
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver;
        solver.compute(*AtA);
        for (int c = 0; c < 3; ++c) {
            int size = keypoints.size();
            vector<double> x(size, 0.0);
            vector<double> delta(width*height, 0.0);
            Eigen::SparseVector<double> ans = solver.solve(*AtB[c]);
            for(Eigen::SparseVector<double>::InnerIterator it(ans); it; ++it)
			    x[it.index()] = it.value();
            double mean = 0.0, max = -256, min = 256;
            for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
                    double val = 0.0;
                    for (auto mv: interp[i*width + j]) {
                        val += x[mv.first] * mv.second;
                    }
                    mean += val;
                    max = std::max(max, val);
                    min = std::min(min, val);
                    delta[i*width+j] = val;
                }
            }
            mean /= (height*width);
            for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
                    double d = delta[i*width+j];
                    int val = std::round(copy_img->get_color(i, j, c) + d - mean);
                    // int val = std::round(copy_img->get_color(i, j, c) + d);
                    qt_img->get_pixel(i, j)[c] = std::max(std::min(val, 255), 0);
                    // reduce to [0, 255] for stability and positive constraints
                    qt_delta_img->get_pixel(i, j)[c] = 255 * (d - min) / (max - min);
                }
            }
        }
    }

    void save_quadtree_fusion_image(const string& dir) const {
        qt_img->save(dir + "/quad.png");
        qt_delta_img->save(dir + "/quad_delta.png");
        root->save(dir + "/tree.png", width, height);
    }

    void report_error() {
        // average per-pixel root-mean-square(RMS) error in R channel
        double sum = 0.0;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                double delta = qt_img->get_color(i, j, 0) - full_img->get_color(i, j, 0);
                sum += delta * delta;
            }
        }
        cout << "RMS error over all pixels on R channel: " << fixed << setprecision(3) << sum / (height * width) << endl;
        // max error on R channel
        double max = -100.0;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                double delta = qt_img->get_color(i, j, 0) - full_img->get_color(i, j, 0);
                max = std::max(max, (delta < 0 ? -delta : delta));
            }
        }
        cout << "max 1-norm error over all pixels on R channel: " << fixed << setprecision(3) << max << endl;
    }
};

#endif // __FUSION_H__
