#include "raytracer.hpp"
#include <atlas/core/Timer.hpp>     // Timer header

// colours
#define NICEGREY Colour { 0.867, 0.867, 0.875 }
#define NICEBLUE Colour{ 0.447, 0.631, 0.898 }
#define NICEPURPLE Colour{ 0.596, 0.514, 0.898 }
#define DEEPPURPLE Colour{0.31,0.17,0.33}
#define RED Colour{ 1, 0.3, 0.3 }
#define DIRT Colour{ 0.6, 0.46, 0.32 }
#define PINK Colour{1, 0.4, 0.85}
#define FADEDPINK Colour{0.93,0.77,0.85}
#define ORANGE Colour{0.93,0.62,0.18}
#define YELLOW Colour{1, 0.8, 0}
#define GREEN Colour{ 0, 0.8, 0.26}
#define TURQUOISE Colour{ 0.314, 0.788, 0.808 }
#define TEAL Colour{0.15,0.82,0.89}
#define BLUE Colour{ 0, 0.54, 0.9 }
#define SKYBLUE Colour{0.77,0.82,0.93}
#define PURPLE Colour{ 0.54, 0.1, 1}
#define GREY Colour{ 0.46, 0.53, 0.60 }

int main() {
    int width = 1000;
    int height = 1000;
    std::vector<Colour> image { (size_t) (width * height) };

    // render(width, height, &image);
    renderParallel(width, height, &image);

    // generate BMP image file
    auto output_path{ "render.bmp" };
    saveToBMP(output_path, width, height, image);

    return 0;
}

void render(int width, int height, std::vector<Colour>* image) {
    std::cout << "Running on 1 thread." << std::endl;
    World w;
    w.width = width;
    w.height = height;
    w.image = image;
    buildScene(w);

    PinholeCamera camera {{50,-150,400}, {0,-25,-100}, 500, &w};
    // FishEye camera { {50,-50,400}, {0,-75,-100}, 425, &w, 180 };

    atlas::core::Timer<float> timer;
    timer.start();

    camera.render();

    float elapsed = timer.elapsed();
    std::cout << "Render time: " << elapsed << " seconds\n";
}

void renderParallel(int width, int height, std::vector<Colour>* image) {
    std::cout << "Running " << std::thread::hardware_concurrency() << " threads in parallel." << std::endl;
    int numThreads = std::thread::hardware_concurrency();
    std::mutex image_mutex;

    int numSlabs = 4 * numThreads;
    Slab slabs[numSlabs];
    std::queue<Slab> workQueue;
    std::mutex workMutex;
    int slabHeight = height / numSlabs;
    for (int i = 0 ; i < numSlabs; i++) {
        int startY = i * slabHeight;
        int startX = 0;
        int endY = startY + slabHeight;
        int endX = width;
        slabs[i] = {startY, startX, endY, endX};
        workQueue.push(slabs[i]);
    }

    atlas::core::Timer<float> timer;
    timer.start();

    std::thread threads[numThreads];
    World worlds[numThreads];
    PinholeCamera cameras[numThreads];
    for (int i = 0; i < numThreads; i++) {
        worlds[i].width = width;
        worlds[i].height = height;
        worlds[i].image = image;
        buildScene(worlds[i]);
        cameras[i] = PinholeCamera({50,-150,400}, {0,-25,-100}, 500, &worlds[i]);
        // FishEye camera { {50,-50,400}, {0,-75,-100}, 425, &w, 180 };
        threads[i] = std::thread(&PinholeCamera::renderSlab, cameras[i], std::ref(image_mutex), std::ref(workMutex), std::ref(workQueue));
    }

    for (int i = 0; i < numThreads; i++) {
        threads[i].join();
    }

    float elapsed = timer.elapsed();
    std::cout << "Render time: " << elapsed << " seconds\n";
}

void buildScene(World& w) {
    w.maxDepth = 4; // number of ray bounces for reflective surfaces
    w.tracer = std::shared_ptr<Tracer>(new Tracer(&w));
    w.background = NICEBLUE;
    w.numSamples = 256;  // must be a square root-able

    // define my objects and materials - note y axis is flipped due to Windows
    std::shared_ptr<Sampler> s7(new Sampler( w.numSamples, 1 ));
    std::shared_ptr<GlossySpecular> gs5(new GlossySpecular( 0.2, 2000, s7, WHITE ));
    std::shared_ptr<PerfectSpecular> ps2(new PerfectSpecular( 0.1, WHITE ));
    std::shared_ptr<PerfectTransmitter> pt1(new PerfectTransmitter( 0.9, 1.5 ));
    std::shared_ptr<Transparent> trans1(new Transparent( WHITE, 0.0, 0.0, gs5, ps2, pt1 ));
    std::shared_ptr<Sphere> sphere1(new Sphere( {-75, -50, 100}, 50, trans1 ));

    std::shared_ptr<Matte> matte2(new Matte( DIRT, 0.4, 0.5 ));
    std::shared_ptr<Plane> plane(new Plane( {0,-1,0}, {0,0,0}, matte2 ));

    std::shared_ptr<Sampler> s1(new Sampler( w.numSamples, 1 ));
    std::shared_ptr<GlossySpecular> gs(new GlossySpecular( 0.7, 100, s1, WHITE ));
    std::shared_ptr<Specular> spec1(new Specular( TURQUOISE, 0.5, 0.6, gs ));
    std::shared_ptr<Triangle> triangle(new Triangle( {-200,0,0}, {-25,-175,-250}, {200,-25,0}, spec1 ));

    std::shared_ptr<Sampler> s2(new Sampler( w.numSamples, 1 ));
    std::shared_ptr<GlossySpecular> gs2(new GlossySpecular( 0.3, 100, s2, WHITE ));
    std::shared_ptr<Sampler> s3(new Sampler( w.numSamples, 100 ));
    std::shared_ptr<GlossySpecular> gs3(new GlossySpecular( 0.9, 100, s3, YELLOW ));
    std::shared_ptr<GlossyReflective> gr1(new GlossyReflective( WHITE, 0.0, 0.0, gs2, gs3 ));
    std::shared_ptr<Sphere> sphere2(new Sphere( {125, -90, -25}, 90, gr1 ));

    std::shared_ptr<Sampler> s6(new Sampler( w.numSamples, 1 ));
    std::shared_ptr<GlossySpecular> gs4(new GlossySpecular( 0.9, 100, s6, WHITE ));
    std::shared_ptr<PerfectSpecular> ps1(new PerfectSpecular( 0.75, WHITE ));
    std::shared_ptr<Reflective> mirror1(new Reflective( BLACK, 0.5, 0.5, gs4, ps1));
    std::shared_ptr<Sphere> sphere3(new Sphere( {500, -200, -500}, 200, mirror1 ));

    w.objects = std::vector<std::shared_ptr<Object>> { sphere1, sphere2, triangle, plane, sphere3 };

    // define my lights
    std::shared_ptr<Sampler> s4(new Sampler( w.numSamples, 1 ));
    std::shared_ptr<AmbientOccluder> ambient(new AmbientOccluder( WHITE, 1.65, s4, 0.0 ));
    std::shared_ptr<DirectionalLight> directional(new DirectionalLight( WHITE , 8.5, {150, 50, -50} ));
    w.ambient = ambient;
    w.lights.push_back(directional);
}

/**
 * Saves a BMP image file based on the given array of pixels. All pixel values
 * have to be in the range [0, 1].
 *
 * @param filename The name of the file to save to.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param image The array of pixels representing the image.
 */
void saveToBMP(std::string const& filename,
               std::size_t width,
               std::size_t height,
               std::vector<Colour> const& image) {
    std::vector<unsigned char> data(image.size() * 3);

    for (std::size_t i{ 0 }, k{ 0 }; i < image.size(); ++i, k += 3)
    {
        Colour pixel = image[i];
        data[k + 0] = static_cast<unsigned char>(pixel.r * 255);
        data[k + 1] = static_cast<unsigned char>(pixel.g * 255);
        data[k + 2] = static_cast<unsigned char>(pixel.b * 255);
    }

    stbi_write_bmp(filename.c_str(),
                   static_cast<int>(width),
                   static_cast<int>(height),
                   3,
                   data.data());
}

