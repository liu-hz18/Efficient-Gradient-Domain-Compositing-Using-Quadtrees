// Efficient Image Fusion using QuadTree implementation
 
// only work under Linux systems and GNU g++
// make
// make example1
// make example2
#include <cstdio>
#include <cstring>
#include <string>
#include <algorithm>
#include <ctime>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "cmdline.h"
#include "fusion.h"

using namespace std;

cmdline::parser init_parser() {
    cmdline::parser argparser;
    argparser.add<string>("source", 's', "source image to be embedded into target image.", true);
    argparser.add<string>("target", 't', "target image.", true);
    argparser.add<string>("mask", 'm', "mask image. (1 channel)");
    argparser.add<int>("xoffset", 'x', "x-dim offset to composite.", true);
    argparser.add<int>("yoffset", 'y', "y-dim offset to composite.", true);
    
    argparser.add<string>("resultdir", 'r', "directory to save results.", false, "results/");
    return argparser;
};

void makedir(const string& dir) {
    if (0 != access(dir.c_str(), 0)) {
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    }
}

int main (int argc, char **argv) {
    std::srand(std::time(0)); // use current time as seed for random generator
    cmdline::parser parser = init_parser();
    parser.parse_check(argc, argv);

    cout << "args: " << endl;
    cout << "\t source image: " << parser.get<string>("source") << endl;
    cout << "\t target image: " << parser.get<string>("target") << endl;
    cout << "\t mask   image: " << parser.get<string>("mask") << endl;
    cout << "\t x offset: " << parser.get<int>("xoffset") << endl;
    cout << "\t y offset: " << parser.get<int>("yoffset") << endl;
    cout << "\t directory to save results: " << parser.get<string>("resultdir") << "/" << endl << endl;

    makedir(parser.get<string>("resultdir"));
    Image* source_img = new Image(parser.get<string>("source"), 3);
    Image* target_img = new Image(parser.get<string>("target"), 3);
    Image* mask_img = new Image(parser.get<string>("mask"), 1);

    ImageFusion fusor(source_img, target_img, mask_img, parser.get<int>("xoffset"), parser.get<int>("yoffset"));
    
    fusor.apply_simple_copy();
    fusor.save_copy_image(parser.get<string>("resultdir"));

    {
        auto t1 = clock();
        fusor.apply_gradient_domain_full();
        auto t2 = clock();
        fusor.save_full_fusion_image(parser.get<string>("resultdir"));
        printf("time elasped (full): %.2fs\n\n", (t2 - t1) / (double)(CLOCKS_PER_SEC));
    }
    
    {
        auto t1 = clock();
        fusor.apply_gradient_domain_quadtree();
        auto t2 = clock();
        fusor.save_quadtree_fusion_image(parser.get<string>("resultdir"));
        printf("time elasped (quadtree): %.2fs\n\n", (t2 - t1) / (double)(CLOCKS_PER_SEC));
    }
    
    fusor.report_error();
    return 0;
}
