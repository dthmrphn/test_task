#include <fstream>
#include <random>

static void gentestfile(const std::string& src, float min, float max,
                        size_t lines) {
    std::fstream f(src);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distr(min, max);

    for (size_t i = 0; i < lines; ++i) {
        f << distr(gen) << "\n";
    }
}

int main() {
    gentestfile("../measures/humidity.txt", 0.0, 1.0, 100);
    gentestfile("../measures/temperature.txt", 15.0, 30.0, 100);
    return 0;
}
