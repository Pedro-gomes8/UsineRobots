#include "../include/RealSense.hpp"

int main() {
    RealSense camera;

    // Activate camera only when needed
    if (!camera.start()) {
        std::cerr << "Failed to start camera\n";
        return -1;
    }

    // Command from some external logic: "scan now!"
    int count = camera.scan();
    std::cout << "Found " << count << " objects.\n";

    // If we found any objects, let's get data from the first one
    if (count > 0) {
        DetectedObject obj = camera.getObject(0);

        std::cout << "Object #0: color=" << obj.color << "\n"
                  << " shape=" << ((obj.shape == ShapeType::CIRCLE) ? "Circle" : "Square") << "\n"
                  << " 2D center=(" << obj.cx << "," << obj.cy << ")\n"
                  << " avgDepth=" << obj.avgDepth << " m\n"
                  << " 3D camera coords=(" << obj.Xc << "," << obj.Yc << "," << obj.Zc << ")\n";

        if (obj.shape == ShapeType::SQUARE) {
            std::cout << " orientation=" << obj.orientation << " deg"
                      << " w=" << obj.width
                      << " h=" << obj.height << "\n";
        } else {
            std::cout << " circle radius=" << obj.radius << " px\n";
        }
    }

    // Stop camera
    camera.stop();
    return 0;
}
