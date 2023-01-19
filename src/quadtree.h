#ifndef __QUADTREE_H__
#define __QUADTREE_H__

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>
#include "image.h"
using namespace std;

class QuadTree {
public:
    int range;
    int xl, xr, yl, yr;
    QuadTree *ch_ll, *ch_lr, *ch_rl, *ch_rr;

    QuadTree(int xl, int xr, int yl, int yr): xl(xl), xr(xr), yl(yl), yr(yr) {
        range = xr - xl;
        ch_ll = ch_lr = ch_rl = ch_rr = nullptr;
    }
    ~QuadTree() {
        if (!isleaf()) {
            delete ch_ll;
            delete ch_lr;
            delete ch_rl;
            delete ch_rr;   
        }
    }
    bool isleaf() const { return ch_ll == nullptr; }
    bool innode(int x, int y) const {
        return (x >= xl && y >= yl) && (x < xr && y < yr);
    }
    bool innode_strict(int x, int y) const { 
        return (x >= xl && y >= yl) && (x <= xr && y <= yr);
    }
    bool on_seam(int x, int y) {
        QuadTree* leaf = find_rightstrict(x, y);
        if (leaf && leaf->xl == x && leaf->yl == y) {
            QuadTree* leaf_left = find_leftstrict(x, y);
            if (leaf_left && ((leaf_left->xr == x && leaf_left->yr == y) || x == 0 || y == 0)) return true;
            else return false; 
        } else return false;
    }
    QuadTree* find_rightstrict(int x, int y) {
        if (!innode(x, y)) return nullptr;
        if (isleaf()) return this;
        int xm = (xl + xr) / 2;
        int ym = (yl + yr) / 2;
        QuadTree* child;
        if (x < xm) child = (y < ym) ? ch_ll : ch_lr;
        else child = (y < ym) ? ch_rl : ch_rr;
        return child->find_rightstrict(x, y);
    }
    QuadTree* find_leftstrict(int x, int y) {
        if (!innode_strict(x, y)) return nullptr;
        if (isleaf()) return this;
        int xm = (xl + xr) / 2;
        int ym = (yl + yr) / 2;
        QuadTree* child;
        if (x <= xm) child = (y <= ym) ? ch_ll : ch_lr;
        else child = (y <= ym) ? ch_rl : ch_rr;
        return child->find_leftstrict(x, y);
    }
    void new_childs() {
        int xm = (xl + xr) / 2;
        int ym = (yl + yr) / 2;
        ch_ll = new QuadTree(xl, xm, yl, ym);
        ch_lr = new QuadTree(xl, xm, ym, yr);
        ch_rl = new QuadTree(xm, xr, yl, ym);
        ch_rr = new QuadTree(xm, xr, ym, yr);
    }
    void split_inner(QuadTree *root, int x, int y, int range) {
        QuadTree* now = root;
        QuadTree* node;
        while(now->range > range) {
            if (now->isleaf()) {
                now->new_childs();
                node = root->find_rightstrict(now->xl-1, now->yl);
                if (node && node->range > now->range) split_inner(root, now->xl-1, now->yl, now->range);
                node = root->find_rightstrict(now->xl, now->yl-1);
                if (node && node->range > now->range) split_inner(root, now->xl, now->yl-1, now->range);
                node = root->find_rightstrict(now->xr, now->yl);
                if (node && node->range > now->range) split_inner(root, now->xr, now->yl, now->range);
                node = root->find_rightstrict(now->xl, now->yr);
                if (node && node->range > now->range) split_inner(root, now->xl, now->yr, now->range);
            }
            int xm = (now->xl + now->xr) / 2;
            int ym = (now->yl + now->yr) / 2;
            QuadTree* child;
            if (x < xm) child = (y < ym) ? now->ch_ll : now->ch_lr;
            else child = (y < ym) ? now->ch_rl : now->ch_rr;
            now = child;
        }
    }
    void split(int x, int y, int range) {
        split_inner(this, x, y, range);
        if (x > xl && y > yl) split_inner(this, x-1, y-1, range);
    }
    template<class Callback>
    void traverse(const Callback& callback) {
        if (isleaf()) callback(xl, xr, yl, yr);
        else {
            ch_ll->traverse(callback);
            ch_lr->traverse(callback);
            ch_rl->traverse(callback);
            ch_rr->traverse(callback);
        }
    }
    void save(const string& path, int width, int height) {
        // width = std::max(width, range);
        // height = std::max(height, range);
        Image img(width, height, 3);
        traverse(
            [&](int xl, int xr, int yl, int yr) {
                xr = std::min(xr, height-1);
                yr = std::min(yr, width-1);
                int color = std::rand();
                for (int i = xl; i < xr; ++i) {
                    for (int j = yl; j < yr; ++j) {
                        uint8_t* ptr = img.get_pixel(i, j);
                        ptr[0] = uint8_t(color);
                        ptr[1] = uint8_t(color >> 8);
                        ptr[2] = uint8_t(color >> 16);
                    }
                }
            }
        );
        img.save(path);
    }
};

#endif // __QUADTREE_H__
