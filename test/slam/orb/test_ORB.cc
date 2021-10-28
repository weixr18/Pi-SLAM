#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
 
int main()
{
	Mat img1 = imread("left.jpg",0);
	Mat img2 = imread("right.jpg",0);
 	std::vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	Ptr<ORB> orb = ORB::create();
 
	// Oriented FAST 特征点
	orb->detect(img1, keypoints1);
	orb->detect(img2, keypoints2);
	// BRIEF 描述子
	orb->compute(img1, keypoints1, descriptors1);
	orb->compute(img2, keypoints2, descriptors2);
 
	// 匹配，使用 Hamming 距离
	std::vector<DMatch> matches;
	BFMatcher bfmatcher(NORM_HAMMING);
	bfmatcher.match(descriptors1, descriptors2, matches);
 
	// 5 匹配对筛选
	double min_dist = 1000, max_dist = 0;
	for (int i = 0; i < descriptors1.rows; i++)
	{
		double dist = matches[i].distance;//汉明距离在matches中
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	// 当描述子之间的匹配大于2倍的最小距离时，即认为该匹配是一个错误的匹配。
	// 但有时描述子之间的最小距离非常小，可以设置一个经验值作为下限
	std::vector<DMatch> good_matches;
	for (int i = 0; i < descriptors1.rows; i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 30.0))
			good_matches.push_back(matches[i]);
	}
 
	// 6 绘制匹配结果
	Mat img_match;
	drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_match);
	imshow("img_match", img_match);
    waitKey();
	return 0;
}
