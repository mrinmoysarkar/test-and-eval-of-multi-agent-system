    #include <iostream>
    #include <pcl/common/common.h>
    #include <pcl/common/angles.h>

    int main(int argc, char **argv) {
        std::cout << "Test rad2deg and deg2rad functions" << std::endl;
    float alpha = 0.25;
    float beta;
        std::cout << "angle in radian is : " << alpha << std::endl;
    beta = pcl::rad2deg(alpha);
        std::cout << "angle in degree is : " << beta << std::endl;
    alpha = pcl::deg2rad(beta);
        std::cout << "angle in radian is : " << alpha << std::endl;
    return 0;
    }
