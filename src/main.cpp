#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

std::vector<std::string> split(const std::string &str, char sep)
{
    std::vector<std::string> v;
    std::stringstream ss(str);
    std::string buffer;
    while( std::getline(ss, buffer, sep) ) {
        v.push_back(buffer);
    }
    return v;
}

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    // user input value to four float valu 0.0 - 100.0, format example: 2.1,50.8,80.0,80.0
    // receive user input value on std::cin
    // check user input value
    // if user input value is not 0.0 - 100.0, ask user input value again until user input value is 0.0 - 100.0
    // if unmber of the user input value is not 4, ask user input value again until user input value is 4
    // if user input value is 0.0 - 100.0, pass the user input to the start_x, start_y, end_x, end_y
    // start code
    float start_x, start_y, end_x, end_y;
    std::string input;
    std::vector<std::string> input_vector;
    bool is_input_valid = false;
    while (!is_input_valid) {
        std::cout << "Please input start_x, start_y, end_x, end_y value (0.0 - 100.0) with comma separated: ";
        std::getline(std::cin, input);
        input_vector = split(input, ',');
        if (input_vector.size() != 4) {
            std::cout << "Please input 4 values with comma separated." << std::endl;
            continue;
        }
        try {
            start_x = std::stof(input_vector[0]);
            start_y = std::stof(input_vector[1]);
            end_x = std::stof(input_vector[2]);
            end_y = std::stof(input_vector[3]);
        } catch (const std::invalid_argument& e) {
            std::cout << "Please input 4 values with comma separated." << std::endl;
            continue;
        }
        if (start_x <= 0.0 || start_x >= 100.0 || start_y <= 0.0 || start_y >= 100.0 || end_x <= 0.0 || end_x >= 100.0 || end_y <= 0.0 || end_y >= 100.0) {
            std::cout << "Please input 4 values with comma separated." << std::endl;
            continue;
        }
        is_input_valid = true;
    }
    // end code

    // std::string input;
    // std::vector<std::string> vector_input;
    // std::vector<float> num_input;
    // float start_x, start_y, end_x, end_y;
    // std::cout << "Input start x, start y, end x, end y " << "\n";
    // std::cout << "variable type: floats, range: 0.0 - 100.0, format example: 2.1,50.8,80.0,80.0" << "\n";
    // bool input_check = true;
    // while (input_check) {
    //     std::cin >> input;
    //     if (input.length() > 0) {
    //         vector_input = split(input, ',');
    //         if (vector_input.size() == 4) {
    //             auto i = 0;
    //             for (const std::string& string_num : vector_input) {
    //                 num_input.push_back(std::stof(string_num));
    //                 if(num_input[i] < 0.0 || num_input[i] > 100.0) {
    //                     std::cout << "The input variable is out of range. once again" << "\n";
    //                     break;
    //                 }
    //                 i++;
    //             }
    //             break;
    //         }
    //         else {
    //             std::cout << "The input variable number is many or few. once again." << "\n";
    //         }
    //     }
    //     else {
    //         std::cout << "Cannot read variable. once again." << "\n";
    //     }
    // }

    // start_x = num_input[0];
    // start_y = num_input[1];
    // end_x = num_input[2];
    // end_y = num_input[3];

    std::cout << "start program" << "\n";
    std::cout << "start: (" << start_x << ", " << start_y << ")" << "\n";
    std::cout << "end: (" << end_x << ", " << end_y << ")" << "\n";
 


    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    // RoutePlanner route_planner{model, 10, 10, 90, 90};
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
