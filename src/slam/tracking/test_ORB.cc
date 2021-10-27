#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
 
int main()
{
	//0读取图像
	Mat img1 = imread("..\\image\\left.jpg",0);
	Mat img2 = imread("..\\image\\right.jpg",0);
 
	// 1 初始化特征点和描述子,ORB
	std::vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	Ptr<ORB> orb = ORB::create();
 
	// 2 提取 Oriented FAST 特征点
	orb->detect(img1, keypoints1);
	orb->detect(img2, keypoints2);
 
	// 3 根据角点位置计算 BRIEF 描述子
	orb->compute(img1, keypoints1, descriptors1);
	orb->compute(img2, keypoints2, descriptors2);
 
	// 4 对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
	vector<DMatch> matches;
	BFMatcher bfmatcher(NORM_HAMMING);
	bfmatcher.match(descriptors1, descriptors2, matches);
 
	// 5 匹配对筛选
	double min_dist = 1000, max_dist = 0;
	// 找出所有匹配之间的最大值和最小值
	for (int i = 0; i < descriptors1.rows; i++)
	{
		double dist = matches[i].distance;//汉明距离在matches中
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	// 当描述子之间的匹配大于2倍的最小距离时，即认为该匹配是一个错误的匹配。
	// 但有时描述子之间的最小距离非常小，可以设置一个经验值作为下限
	vector<DMatch> good_matches;
	for (int i = 0; i < descriptors1.rows; i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 30.0))
			good_matches.push_back(matches[i]);
	}
 
	// 6 绘制匹配结果
	Mat img_match;
	drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_match);
 
	return 0;
}
