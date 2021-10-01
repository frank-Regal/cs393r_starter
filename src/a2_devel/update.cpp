#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <random>
#include <chrono>
#include <fstream>
#include <iomanip>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


// the intent here is to trim away any of the map sections which are not within lidar range
// [0] is x coordinate
// [1] is y coordinate
// [i] is map point number in list
void reduce_map(Eigen::Vector3d particle_state, std::string map)
{
    // INPUT:
    // Vector3d partical_state: [x, y, theta] of the predicted forward particle
    // <.txt> file for the map, with each row formatted [x1 y2 x2 y2]

    // Parse Map TODO
    // Vector2d map_coordinate_1: [x,y] location of x1, y1 in row i from <map.txt> file



    // Vector2d map_coordinate_2: [x,y] location of x2, y2 in row i from <map.txt> file

}


int main(void){

    std::ifstream in_file;

    int num;
    double total;

    double x1;
    double y1;
    double x2;
    double y2;

    // Open text file
    in_file.open("/Users/regal/devel/github_frank_Regal/autonomous_robot/cs393r_starter/src/a2_devel/test.txt");

    if (!in_file) {
        std::cerr << "Problem opening file" << std::endl;
        return 1;
    }

    // while (in_file >> x1 >> y1 >> x2 >> y2)
    // {
    //     std::cout << std::setw(10) << std::left << x1
    //                    << std::setw(10) << y1 
    //                    << std::setw(10) << x2
    //                    << std::setw(10) << y2
    //                    << std::endl;
    // }
    char c;
    std::string line{};
    std::string number{};
    int iter {0};
    Eigen::Matrix2d map_lines;
    // std::vector <std::vector<double> holder {};
    std::vector <std::string> data {};
    while (std::getline(in_file, line)) 
    {
        line.erase(remove(line.begin(),line.end(),' '),line.end());
        std::cout << line << std::endl;
        for (int i {0}; i < line.length(); i++)
        {
            if (line[i] == ',')
            {   
                line.erase(line.begin()+i);
                //std::cout << number << std::endl;
                data.push_back(number);
                number.clear();

            }
            number = number + line[i];
        }
        std::cout << line << std::endl;
        std::cout << std::endl;
        data.push_back(number);
        number.clear();

    }

    in_file.close();

    for (auto it: data)
        std::cout << it << std::endl;
    

    
    return 0;
}