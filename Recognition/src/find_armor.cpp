#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "Angel_Calculate.h"
#include "find_armor.h"



ArmorBox::ArmorBox(const LightBlobs &blobs, int color, const ArmorType &Armor_type) : light_blobs(blobs), box_color(color), type(Armor_type) {};

// 判断两个灯条的角度差
static bool angelJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle : light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle : light_blob_j.rect.angle - 90;
    return abs(angle_i - angle_j) < 80;
}
// 判断两个灯条的高度差
static bool heightJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    return abs(centers.y) < 50;
}
// 判断两个灯条的间距
static bool lengthJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    double side_length;
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
    return (side_length / light_blob_i.length < 5 && side_length / light_blob_i.length > 0.2);
}
// 判断两个灯条的长度比
static bool lengthRatioJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    return (light_blob_i.length / light_blob_j.length < 3 && light_blob_i.length / light_blob_j.length > 0.2);
}

/* 判断两个灯条的错位度，不知道英文是什么！！！ */
static bool CuoWeiDuJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle : light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle : light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0 / 180.0 * 3.14159265459;
    if (abs(angle_i - angle_j) > 90)
    {
        angle += 3.14159265459 / 2;
    }
    Eigen::Vector2f orientation(cos(angle), sin(angle));
    Eigen::Vector2f p2p(light_blob_j.rect.center.x - light_blob_i.rect.center.x,
                        light_blob_j.rect.center.y - light_blob_i.rect.center.y);
    return abs(orientation.dot(p2p)) < 40;
}
// 判断装甲板方向
static bool boxAngleJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle : light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle : light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0;
    if (abs(angle_i - angle_j) > 90)
    {
        angle += 90.0;
    }
    return (-120.0 < angle && angle < -60.0) || (60.0 < angle && angle < 120.0);
}

// Check if there is another light in the boundingRect formed by the 2 lights
static bool containLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, const LightBlobs &lights)
{
    auto points = std::vector<cv::Point2f>{light_blob_i.top, light_blob_i.bottom, light_blob_j.top, light_blob_j.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto &test_light : lights)
    {
        if (test_light.center == light_blob_i.center || test_light.center == light_blob_j.center)
            continue;

        if (
            bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center))
        {
            return true;
        }
    }

    return false;
}

// 判断两个灯条是否可以匹配
static bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, int enemy_color)
{
    return light_blob_i.blob_color == enemy_color &&
           light_blob_j.blob_color == enemy_color &&
           lengthRatioJudge(light_blob_i, light_blob_j) &&
           lengthJudge(light_blob_i, light_blob_j) &&
           heightJudge(light_blob_i, light_blob_j) ;//&&
           //angelJudge(light_blob_i, light_blob_j);
    //    boxAngleJudge(light_blob_i, light_blob_j) &&
    //    CuoWeiDuJudge(light_blob_i, light_blob_j);
}

void Judge_Type(const LightBlob &light_blob_i, const LightBlob &light_blob_j, ArmorType &type)
{
    // 两个灯条的间距
    double side_length;
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
    // 灯条的高度
    double light_height;
    light_height = (light_blob_i.length + light_blob_j.length) / 2;
    // cout<<" side length: "<<side_length<<endl;
    // cout<<" light_height: "<<light_height<<endl;
    // cout<<" (side_length / light_height) :"<<(side_length / light_height)<<endl;
    // if ((side_length / light_height) > 3.2)
    if ((side_length / light_height) > 2.8)
    {
        type = ArmorType::LARGE;
    }
    else
    {
        type = ArmorType::SMALL;
    }
}

// Returns the order of rectangular points
void order_points_2nd(vector<Point2f> &points)
{
    vector<pair<float, Point2f *>> order_points_x, order_points_y;

    for (int i = 0; i < 4; i++)
    {
        order_points_x.push_back(make_pair(points.at(i).x, &points.at(i)));
        order_points_y.push_back(make_pair(points.at(i).y, &points.at(i)));
    }

    // find the max x and max y
    sort(order_points_x.begin(), order_points_x.end());
    sort(order_points_y.begin(), order_points_y.end());

    // float point
    // 顺序点
    Point2f left_top, left_bottom, right_top, right_bottom;

    // match point
    for (int i = 2; i < 4; i++)
    {
        if ((*order_points_x.at(2).second).y >= (*order_points_x.at(3).second).y)
        {
            right_bottom.x = order_points_x.at(2).first;
            right_bottom.y = (*(order_points_x.at(2).second)).y;
            right_top.x = order_points_x.at(3).first;
            right_top.y = (*(order_points_x.at(3).second)).y;
        }
        else
        {
            right_bottom.x = order_points_x.at(3).first;
            right_bottom.y = (*(order_points_x.at(3).second)).y;
            right_top.x = order_points_x.at(2).first;
            right_top.y = (*(order_points_x.at(2).second)).y;
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if ((*order_points_x.at(0).second).y >= (*order_points_x.at(1).second).y)
        {
            left_bottom.x = order_points_x.at(0).first;
            left_bottom.y = (*(order_points_x.at(0).second)).y;
            left_top.x = order_points_x.at(1).first;
            left_top.y = (*(order_points_x.at(1).second)).y;
        }
        else
        {
            left_bottom.x = order_points_x.at(1).first;
            left_bottom.y = (*(order_points_x.at(1).second)).y;
            left_top.x = order_points_x.at(0).first;
            left_top.y = (*(order_points_x.at(0).second)).y;
        }
    }

    points.clear();

    points.push_back(left_top);
    points.push_back(right_top);
    points.push_back(right_bottom);
    points.push_back(left_bottom);
}

// // // 匹配所有灯条，得出装甲板候选区
Armors Tradition::Find_armor(cv::Mat &src, LightBlobs &light_blobs)
{
    Armors Armors;
    ArmorType type;
    string big_or_small;

    for (size_t i = 0; i < light_blobs.size() - 1; ++i)
    {
        const auto& blob_i = light_blobs[i];
        cv::Rect2d rect_left = blob_i.rect.boundingRect();

        for (size_t j = i + 1; j < light_blobs.size(); ++j)
        {
            const auto& blob_j = light_blobs[j];
            cv::Rect2d rect_right = blob_j.rect.boundingRect();

            if (!isCoupleLight(blob_i, blob_j, enemy_color))
            {
                continue;
            }

            if (containLight(blob_i, blob_j, light_blobs))
            {
                continue;
            }

            Judge_Type(blob_i, blob_j, type);
            big_or_small = ARMOR_TYPE_STR[static_cast<int>(type)];
            /*
                1-----2
                |     |
                0-----3
            */
            if (light_blobs[i].center.x < light_blobs[j].center.x)
            {
                std::vector<cv::Point2f> points = {blob_i.bottom, blob_i.top, blob_j.top, blob_j.bottom};
                // std::vector<cv::Point2f> points = {
                //     Point2f((blob_i.bottom.x*4 + blob_i.top.x),(blob_i.bottom.y*4 + blob_j.bottom.y))/5,
                //     Point2f((blob_i.top.x*4 + blob_i.bottom.x),(blob_i.top.y*4 + blob_j.top.y))/5,
                //     Point2f((blob_j.top.x*4 + blob_j.bottom.x),(blob_j.top.y*4 + blob_i.top.y))/5,
                //     Point2f((blob_j.bottom.x*4 + blob_j.top.x),(blob_j.bottom.y*4 + blob_i.bottom.y))/5
                // };
                Armors.emplace_back(points, enemy_color, type);
            }
            else
            {
                std::vector<cv::Point2f> points = {blob_j.bottom, blob_j.top, blob_i.top, blob_i.bottom};
                // std::vector<cv::Point2f> points = {
                //     Point2f((blob_j.bottom.x*4 + blob_j.top.x),(blob_j.bottom.y*4 + blob_i.bottom.y))/5,
                //     Point2f((blob_j.top.x*4 + blob_j.bottom.x),(blob_j.top.y*4 + blob_i.top.y))/5,
                //     Point2f((blob_i.top.x*4 + blob_i.bottom.x),(blob_i.top.y*4 + blob_j.top.y))/5,
                //     Point2f((blob_i.bottom.x*4 + blob_i.top.x),(blob_i.bottom.y*4 + blob_j.bottom.y))/5
                // };
                Armors.emplace_back(points, enemy_color, type);
            }
        }
    }
    
    light_blobs.clear();
    return Armors;
}

YAML::Node Tradition::readYAML(const std::string& filename){
    try {
        YAML::Node config = YAML::LoadFile(filename);
        enemy_color = config["Find"]["enemy_color"].as<double>();
        is_get_from_ds = config["Find"]["is_get_from_ds"].as<double>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Error: " + std::string(e.what()));
    }
    return YAML::LoadFile(filename);
}

Tradition::Tradition(){
    readYAML(Yaml);
}