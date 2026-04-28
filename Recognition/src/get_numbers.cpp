#include <fstream>
#include "find_armor.h"
#include "Base.h"
NumberClassifier::NumberClassifier(
    const std::string &model_path, const std::string &label_path, const double thre,
    const std::vector<std::string> &ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes)
{
    net_ = cv::dnn::readNetFromONNX(model_path);

    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line))
    {
        class_names_.push_back(line);
    }
}

std::mutex mutex_;

void NumberClassifier::extractNumbers(const cv::Mat &src, std::vector<Armor> &armors)
{

    /*
    1---2
    |   |
    0---3
    */

    // Light length in image
    const int light_length = 12;
    // Image size after warp
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 52;
    // Number ROI size
    const cv::Size roi_size(28, 28);
    const cv::Size input_size(28, 28);
    for (auto &armor : armors)
    {

        // Warp perspective transform
        cv::Point2f lights_vertices[4] = {armor.points[0], armor.points[1], armor.points[2], armor.points[3]};

        const int top_light_y = (warp_height - light_length) / 2 - 1;
        const int bottom_light_y = top_light_y + light_length;
        const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
        string big_or_small = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        // cout<<"ARMOR TYPE:"<<big_or_small<<endl;
        cv::Point2f target_vertices[4] = {
            cv::Point(0, bottom_light_y),
            cv::Point(0, top_light_y),
            cv::Point(warp_width - 1, top_light_y),
            cv::Point(warp_width - 1, bottom_light_y),
        };
        cv::Mat number_image;
        auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
        cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

        // Get ROI
        number_image =
            number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

        // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        // cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        /*
        * gray_image: 这是输入的灰度图像，即待处理的图像。
        * binary_image: 这是输出的二值化图像，函数将在此处存储结果。
        * 255: 这是输出的最大像素值。在二值化过程中，大于阈值的像素将被设置为这个值（在这里是255），小于等于阈值的像素将被设置为0。
        * cv::ADAPTIVE_THRESH_GAUSSIAN_C: 这是自适应阈值化的方法之一。它表示使用高斯加权求和的方法计算邻域的加权平均值作为阈值。
        * cv::THRESH_BINARY: 这是阈值类型。在这个例子中，表示将大于阈值的像素设置为最大值（255），小于等于阈值的像素设置为0。
        * 11: 这是阈值化计算中使用的邻域大小（block size）。它指定了在每个像素周围多大的邻域内计算局部阈值。通常选择一个奇数，比如 3、5、7、11 等。
        * 2: 这是从邻域均值或加权均值中减去的常数（C）。它可以用来微调阈值的结果，以适应不同的图像对比度和光照变化。
        调整建议：
        === blockSize (第六个参数)：增大该值会使得阈值的计算更加平滑，适用于更大尺寸的图像区域。但是也会导致细节信息丢失，因此需要根据具体情况选择一个合适的值。
        === C (第七个参数)：增大该值会使得阈值升高，这意味着更多的像素可能会被视为背景或非目标物体。减小该值则会降低阈值，这样更多的像素会被视为目标物体。
        */
        cv::adaptiveThreshold(number_image, number_image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 19, -3);

        cv::resize(number_image, number_image, input_size);
        // Set the top 2 rows and bottom 2 rows to black (value 0)
        // number_image.rowRange(0, 2).setTo(cv::Scalar(0));  // Top 2 rows
        // number_image.rowRange(number_image.rows - 2, number_image.rows).setTo(cv::Scalar(0));  // Bottom 2 rows

        armor.number_img = number_image;

        // string window = to_string(i);
        // imshow(window,armor.number_img);
        // i++;
    }
}
// 训练用平
string Writepath1;
int counting1 = 7000;
string Photo_name1;

void NumberClassifier::classify(std::vector<Armor> &armors)
{
    for (auto &armor : armors)
    {
        // Normalize
        cv::Mat input = armor.number_img / 255.0;
        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(input, blob);

        // Set the input blob for the neural network
        mutex_.lock();
        net_.setInput(blob);

        // Forward pass the image blob through the model
        cv::Mat outputs = net_.forward().clone();
        mutex_.unlock();

        // cout<<"==== In MLP ===="<<endl;
        //  Do softmax
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        // Decode the output
        double confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = confidence;
        armor.number = class_names_[label_id];

        // std::stringstream result_ss;
        // result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
        //             << armor.confidence * 100.0 << "%";
        // armor.classfication_result = result_ss.str();
    }
    // 检查识别精度
    // for (auto & armor : armors){
    //     cout<<armor.number << ": " << std::fixed << std::setprecision(1)<< armor.confidence * 100.0 << "%"<<endl<<endl;
    // }
         
    armors.erase(
        std::remove_if(
            armors.begin(), armors.end(),
            [this](const Armor &armor)
            {
                bool mismatch_armor_type = false;
                // 检查是否应该忽略这个类别
                if (armor.confidence < threshold)
                {
                    return true;
                }

                for (const auto &ignore_class : ignore_classes_)
                {
                    if (armor.number == ignore_class)
                    {
                        return true;
                    }
                }

#ifdef SENTRY
                if (armor.type == ArmorType::LARGE)
                {
                    mismatch_armor_type =
                        armor.number == "outpost" || armor.number == "guard";
                }
                else if (armor.type == ArmorType::SMALL)
                {
                    mismatch_armor_type = armor.number == "1" || armor.number == "base" || armor.number == "guard";
                }
#else
                if (armor.type == ArmorType::LARGE)
                {
                    mismatch_armor_type =
                        armor.number == "outpost" || armor.number == "guard" || armor.number == "3" || armor.number == "4" || armor.number == "base";
                }
                else if (armor.type == ArmorType::SMALL)
                {
                    mismatch_armor_type = armor.number == "1" || armor.number == "base";
                }
#endif
                return mismatch_armor_type;
            }),
        armors.end());

    // for (auto &armor : armors) {
    //     armor.number_img = cv::Mat(); // 将 number_img 置为空矩阵
    // }

//     // 保存图像
//     for (auto &armor : armors) {
//         if (armor.number == "1") {
//                 Writepath1 = "/home/killer_machine/datasets/1/";
//                 Photo_name1 = Writepath1 + to_string(counting1) + ".jpg";
//                 imwrite(Photo_name1, armor.number_img);
//                 cout << Photo_name1 << endl;
//                 counting1++;
//         }
//         else if (armor.number == "2") {
//             Writepath1 = "/home/killer_machine/datasets/2/";
//             Photo_name1 = Writepath1 + to_string(counting1) + ".jpg";
//             imwrite(Photo_name1, armor.number_img);
//             cout << Photo_name1 << endl;
//             counting1++;
//         }
//         else if (armor.number == "3") {
//             Writepath1 = "/home/killer_machine/datasets/3/";
//             Photo_name1 = Writepath1 + to_string(counting1) + ".jpg";
//             imwrite(Photo_name1, armor.number_img);
//             cout << Photo_name1 << endl;
//             counting1++;
//         }
//         else if (armor.number == "4") {
//             Writepath1 = "/home/killer_machine/datasets/4/";
//             Photo_name1 = Writepath1 + to_string(counting1) + ".jpg";
//             imwrite(Photo_name1, armor.number_img);
//             cout << Photo_name1 << endl;
//             counting1++;
//         }
//         else if (armor.number == "outpost") {
//             Writepath1 = "/home/killer_machine/datasets/outpost/";
//             Photo_name1 = Writepath1 + to_string(counting1) + ".jpg";
//             imwrite(Photo_name1, armor.number_img);
//             cout << Photo_name1 << endl;
//             counting1++;
//         }
//         else if (armor.number == "guard") {
//             Writepath1 = "/home/killer_machine/datasets/guard/";
//             Photo_name1 = Writepath1 + to_string(counting1) + ".jpg";
//             imwrite(Photo_name1, armor.number_img);
//             cout << Photo_name1 << endl;
//             counting1++;
//         }
//         else if (armor.number == "base") {
//             Writepath1 = "/home/killer_machine/datasets/base/";
//             Photo_name1 = Writepath1 + to_string(counting1) + ".jpg";
//             imwrite(Photo_name1, armor.number_img);
//             cout << Photo_name1 << endl;
//             counting1++;
//         }
//         else if (armor.number == "negative") {
//             Writepath1 = "/home/killer_machine/datasets/negative/";
//             Photo_name1 = Writepath1 + to_string(counting1) + ".jpg";
//             imwrite(Photo_name1, armor.number_img);
//             cout << Photo_name1 << endl;
//             counting1++;
//         }
//     }
}

const std::string model_path = "../model/mlp.onnx";
const std::string label_path = "../model/label.txt";
const double threshold1 = 0.7;
const std::vector<std::string> ignore_classes = {"negative"};
NumberClassifier classifier(model_path, label_path, threshold1, ignore_classes);