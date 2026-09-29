#include <algorithm>

#include "opencv2/opencv.hpp"

#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

int main()
{
  // 初始化相机、yolo类
  Camera camera;

  auto_aim::YOLO yolo("./configs/yolo.yaml");

  while (1) {
    // 调用相机读取图像
    cv::Mat img = camera.read();
    if (img.empty()) {
      continue;  // 这一帧没有读到图像，跳过
    }

    // 调用yolo识别opencv标志
    auto armors = yolo.detect(img);

    // 在图像上绘制识别结果
    for (const auto & armor : armors) {
      // 用绿色线条连接装甲板四个关键点（左上、左下、右下、右上），形成闭合矩形
      tools::draw_points(img, armor.points, {0, 255, 0});

      // 在矩形上方中间位置用英文标注灯带颜色和中间数字（字体放大两倍）
      std::string text = auto_aim::COLORS[armor.color] + " " + auto_aim::ARMOR_NAMES[armor.name];
      double font_scale = 2.0;  // 字体放大两倍
      int thickness = 4;        // 线条粗细同步加粗
      // 不依赖关键点的存储顺序，直接取 y 最小的两个点作为矩形上边
      std::vector<cv::Point2f> top_pts = armor.points;
      std::sort(top_pts.begin(), top_pts.end(),
                [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
      float center_x = (top_pts[0].x + top_pts[1].x) / 2;  // 上边中点
      float top_y = top_pts[0].y;
      int baseline = 0;
      cv::Size text_size =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
      cv::Point text_pos(
        static_cast<int>(center_x) - text_size.width / 2,  // 水平居中
        static_cast<int>(top_y) - 10);                     // 贴矩形上边
      tools::draw_text(img, text, text_pos, {0, 255, 0}, font_scale, thickness);
    }

    // 显示图像
    cv::resize(img, img, cv::Size(640, 480));
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  return 0;
}
