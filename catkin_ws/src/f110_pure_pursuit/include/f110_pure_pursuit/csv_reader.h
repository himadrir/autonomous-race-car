//
// Created by yash on 10/20/19.
//

#ifndef SRC_CSV_READER_H
#define SRC_CSV_READER_H

#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

#include "f110_pure_pursuit/types.h"

namespace f110
{

/// A Class to read csv data files
class CSVReader
{
    std::string fileName;
    std::string delimeter;

public:
    explicit CSVReader(std::string filename, std::string delm = ",") :
            fileName(std::move(filename)), delimeter(std::move(delm))
    {}

    ///
    /// Function to fetch data from a CSV File
    std::vector<f110::WayPoint> getData(int n_points)
    {
        std::ifstream file(fileName);
        if (!file)
        {
            throw std::runtime_error("Invalid Path for csv file.");
        }
        std::vector<f110::WayPoint> dataList;

        std::string line = "";
        // Iterate through each line and split the content using delimeter
        while (getline(file, line))
        {
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
            f110::WayPoint way_point{};
            way_point.x = std::stod(vec[0]);
            way_point.y = std::stod(vec[1]);
            way_point.speed = 0.0;

            dataList.emplace_back(way_point);
        }

        std::vector<f110::WayPoint> truncated_dataList;
        size_t increment = dataList.size()/n_points;
        for(size_t i = 0; i < dataList.size(); i = i + increment)
        {
            truncated_dataList.emplace_back(dataList[i]);
        }

        // Close the File
        file.close();

        return truncated_dataList;
    }
};

} // namespace f110

#endif //SRC_CSV_READER_H
