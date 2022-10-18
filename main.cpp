#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>


#define PRE_ERUPTION_FILE "st-helens/pre.data"
#define POST_ERUPTION_FILE "st-helens/post.data"
#define IMAGE_WIDTH 512
#define IMAGE_HEIGHT 512
#define IMAGE_RESOLUTION (IMAGE_WIDTH * IMAGE_HEIGHT)
#define WIDTH_RESOLUTION 30
#define HEIGHT_RESOLUTION 11

#define TEST_P0_X 0
#define TEST_P0_Y 0
#define TEST_P1_X 1
#define TEST_P1_Y 1

// NOTE: Didn't finish the project to completion within the 3 hour allotment, but my solution works for a couple input point pairs, mostly going in the +x, +y directions.
//			I think I have the core algorithm in there, but I did not get a chance to handle all the possible directions and scales the path can take

// INSTRUCTIONS: To use, open up Visual Studio and create a new empty project. Add this file to project source directory, as well as the folder containing the
//					height map data that was included in the assignment (the st. helens data). No additional setup should be necessary to run (i.e. no external libraries)

float calculateDistance_PointToPoint(int x0, int y0, int x1, int y1) {
	unsigned int x0_scaled = x0 * WIDTH_RESOLUTION;
	unsigned int y0_scaled = y0 * WIDTH_RESOLUTION;

	unsigned int x1_scaled = x1 * WIDTH_RESOLUTION;
	unsigned int y1_scaled = y1 * WIDTH_RESOLUTION;

	float x_diff = x1_scaled - x0_scaled;
	float y_diff = y1_scaled - y0_scaled;

	return std::sqrt(x_diff * x_diff + y_diff * y_diff);
}

float calculateDistance_PointToPointWithHeight(int x0, int y0, int x1, int y1, unsigned char* terrain_heights) {
	unsigned int height_0_scaled = (unsigned int)terrain_heights[x0 + y0 * IMAGE_WIDTH] * HEIGHT_RESOLUTION;
	unsigned int x0_scaled = x0 * WIDTH_RESOLUTION;
	unsigned int y0_scaled = y0 * WIDTH_RESOLUTION;

	unsigned int height_1_scaled = (unsigned int)terrain_heights[x1 + y1 * IMAGE_WIDTH] * HEIGHT_RESOLUTION;
	unsigned int x1_scaled = x1 * WIDTH_RESOLUTION;
	unsigned int y1_scaled = y1 * WIDTH_RESOLUTION;


	float x_diff = x1_scaled - x0_scaled;
	float y_diff = y1_scaled - y0_scaled;
	float height_diff = height_1_scaled - height_0_scaled;
	float distance = std::sqrt(x_diff * x_diff + y_diff * y_diff + height_diff * height_diff);

	return distance;
}


float calculateDistance_Surface(int x0, int y0, int x1, int y1, unsigned char* terrain_heights) {

	float p0_x = (float)x0;
	float p0_y = (float)y0;
	float x_1 = (float)x1;
	float y_1 = (float)y1;
	unsigned int p0_height_scaled = (int)terrain_heights[x0 + y0 * IMAGE_WIDTH] * HEIGHT_RESOLUTION;
	float p_isect_x = 0.0f;
	float p_isect_y = 0.0f;
	
	std::vector<std::tuple<std::tuple<unsigned int, unsigned int>, std::tuple<unsigned int,unsigned int>>> edgesToCheck;


	// reverse direction if y is positive (for less duplicated code)
	bool x_positive = x1 - x0 > 0;
	bool y_positive = y1 - y0 > 0;
	if (!y_positive) {
		int tempX = x0;
		int tempY = y0;
		
		p0_x = (float)x1;
		p0_y = (float)y1;
		x0 = x1;
		y0 = y1;
		p0_height_scaled = (int)terrain_heights[x1 + y1 * IMAGE_WIDTH] * HEIGHT_RESOLUTION;

		x1 = tempX;
		y1 = tempY;
		x_1 = (float)tempX;
		y_1 = (float)tempY;
	}

	// get the edges to process
	for (unsigned int yi = y0; yi <= y1; ++yi) {
		for (unsigned int xi = x0; xi <= x1; ++xi) {
			if (!(xi == x0 && yi == y0)) {
				edgesToCheck.push_back(std::tuple<std::tuple<unsigned int, unsigned int>, std::tuple<unsigned int, unsigned int>>(std::tuple<unsigned int, unsigned int>(xi + 1, yi), std::tuple<unsigned int, unsigned int>(xi, yi)));
				edgesToCheck.push_back(std::tuple<std::tuple<unsigned int, unsigned int>, std::tuple<unsigned int, unsigned int>>(std::tuple<unsigned int, unsigned int>(xi, yi + 1), std::tuple<unsigned int, unsigned int>(xi, yi)));
			}
			edgesToCheck.push_back(std::tuple<std::tuple<unsigned int, unsigned int>, std::tuple<unsigned int, unsigned int>>(std::tuple<unsigned int, unsigned int>(xi + 1, yi), std::tuple<unsigned int, unsigned int>(xi, yi + 1)));
		}
	}


	float distance_accum = 0.0f;

	// loop through each of the edges the line from p0 to p1 potentially crosses and accumulate distance at the intersection of the p0-p1 line and the triangle edge
	// line segment - line segment intersection equation from: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	for (int i = 0; i < edgesToCheck.size(); ++i) {

		float x2 = std::get<0>(std::get<0>(edgesToCheck[i]));
		float y2 = std::get<1>(std::get<0>(edgesToCheck[i]));
		float x3 = std::get<0>(std::get<1>(edgesToCheck[i]));
		float y3 = std::get<1>(std::get<1>(edgesToCheck[i]));

		float t = ((p0_x - x2) * (y2 - y3) - (p0_y - y2) * (x2 - x3)) / ((p0_x - x_1) * (y2 - y3) - (p0_y - y_1) * (x2 - x3));
		float u = ((p0_x - x2) * (p0_y - y2) - (p0_y - y2) * (p0_x - x_1)) / ((p0_x - x_1) * (y2 - y3) - (p0_y - y_1) * (x2 - x3));
		if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {

			p_isect_x = p0_x + t * (x_1 - p0_x);
			p_isect_y = p0_y + t * (y_1 - p0_y);

			float p0_x_scaled = p0_x * WIDTH_RESOLUTION;
			float p0_y_scaled = p0_y * WIDTH_RESOLUTION;

			float p_isect_x_scaled = p_isect_x * WIDTH_RESOLUTION;
			float p_isect_y_scaled = p_isect_y * WIDTH_RESOLUTION;

			unsigned int height_value_2 = (unsigned int)terrain_heights[(int)x2 + (int)y2 * IMAGE_WIDTH] * HEIGHT_RESOLUTION;
			unsigned int height_value_3 = (unsigned int)terrain_heights[(int)x3 + (int)y3 * IMAGE_WIDTH] * HEIGHT_RESOLUTION;

			// interpolated height values from p2 and p3 based on u value from line intersection equation (lerp)
			float p_isect_height = (1.0 - u) * height_value_2 + u * height_value_3;

			float x_diff = p_isect_x_scaled - p0_x_scaled;
			float y_diff = p_isect_y_scaled - p0_y_scaled;
			float height_diff = p_isect_height - p0_height_scaled;

			distance_accum += std::sqrt(x_diff * x_diff + y_diff * y_diff + height_diff * height_diff);

			p0_x = p_isect_x;
			p0_y = p_isect_y;
			p0_height_scaled = p_isect_height;
		}
	}


	return distance_accum;
}

int main() {
	unsigned char file_heights[IMAGE_RESOLUTION];
	std::ifstream input_file;
	input_file.open(PRE_ERUPTION_FILE, std::ios::binary);
	for (int i = 0; i < IMAGE_RESOLUTION; ++i) {
		file_heights[i] = input_file.get();
		if (i == TEST_P0_X + TEST_P0_Y * IMAGE_WIDTH) {
			std::cout << "Point 0 Raw Height Value: " << (int)file_heights[i] << std::endl;
		}
		else if (i == TEST_P1_X + TEST_P1_Y * IMAGE_WIDTH) {
			std::cout << "Point 1 Raw Height Value: " << (int)file_heights[i] << std::endl;
		}
	}
	std::cout << "Point-to-Point Distance: " << calculateDistance_PointToPoint(TEST_P0_X, TEST_P0_Y, TEST_P1_X, TEST_P1_Y) << std::endl;
	std::cout << "Point-to-Point (with height) Distance: " << calculateDistance_PointToPointWithHeight(TEST_P0_X, TEST_P0_Y, TEST_P1_X, TEST_P1_Y, file_heights) << std::endl;
	std::cout << "Surface Distance: " << calculateDistance_Surface(TEST_P0_X, TEST_P0_Y, TEST_P1_X, TEST_P1_Y, file_heights) << std::endl;

	
}