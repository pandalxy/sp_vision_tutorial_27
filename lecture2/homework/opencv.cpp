#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
  // 初始化相机、yolo类
  Camera camera;
  // 课程包中目前只有装甲板模型；如需识别 opencv 标志，拿到对应模型后替换这里的配置路径即可
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

      // 在矩形上方用英文标注灯带颜色和中间数字
      std::string text = auto_aim::COLORS[armor.color] + " " + auto_aim::ARMOR_NAMES[armor.name];
      cv::Point text_pos = armor.points[0];
      text_pos.y -= 10;
      tools::draw_text(img, text, text_pos, {0, 255, 0});
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
