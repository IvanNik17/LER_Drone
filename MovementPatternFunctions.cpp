// MovementPatternFunctions.cpp : Defines the entry point for the console application.
//

/*
#include <iostream>
#include <vector>
#include <math.h>
#include <complex> 
#include <algorithm> 
#include <numeric>
#include <tuple>

#include <Eigen/Dense>
*/
#include "UDG_Hokuyo.h"
using namespace std;
using namespace Eigen;


const float PI = 3.14159265;



void scanningPattern_rotateToAngle(vector<float>& x_pos, vector<float>& y_pos, float xC, float yC, float rotationAngle){

	rotationAngle = rotationAngle*(PI / 180);

	for (int i = 0; i < static_cast<int>(x_pos.size()); i++)
	{
		x_pos[i] = cos(rotationAngle) * (x_pos[i] - xC) - sin(rotationAngle) * (y_pos[i] - yC) + xC;
		y_pos[i] = sin(rotationAngle) * (x_pos[i] - xC) + cos(rotationAngle) * (y_pos[i] - yC) + yC;
	}

}

void scanningPattern_horiz(vector<float>& x_vec, vector<float>& y_vec, vector<float>& z_vec, float xC, float yC, float radius, float height, float rotAngle, float startAngle, float endAngle, float deltaPoints){

	
	if (startAngle > endAngle){
		deltaPoints = -deltaPoints;
	}

	endAngle += deltaPoints;

	for (float i = startAngle; i < endAngle; i = i+ deltaPoints)
	{
		x_vec.push_back(xC + radius*cos(i*(PI / 180)));
		y_vec.push_back(yC + radius*sin(i*(PI / 180)));
		z_vec.push_back(height);
	}

	scanningPattern_rotateToAngle(x_vec, y_vec, xC, yC, rotAngle);

}


//float calculatePCA_rotAngle(float positions[][2], int size){
float calculatePCA_rotAngle(vector<double>& x_pos, vector<double>& y_pos){
	
	int size = x_pos.size();

	MatrixXf posMat(x_pos.size(), 2);

	for (int i = 0; i < size; i++)
	{
		posMat(i, 0) = x_pos[i];
		posMat(i, 1) = y_pos[i];
	}

	/*
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			posMat(i, j) = positions[i][j];
		}
	}
	*/



	float avg_x = 0.0;
	float sum_x = 0.0;
	float avg_y = 0.0;
	float sum_y = 0.0;
	for (int i = 0; i < size; i++)
	{
		sum_x += posMat(i, 0);
		sum_y += posMat(i, 1);
	}
	avg_x = ((float)sum_x) / size;
	avg_y = ((float)sum_y) / size;

	for (int i = 0; i < size; i++)
	{
		posMat(i, 0) -= avg_x;
		posMat(i, 1) -= avg_y;
	}

	int fact = size - 1;

	MatrixXf covMat(2, 2);
	covMat = (posMat.transpose()*posMat.conjugate()) / fact;
	
	/*
	std::cout << covMat(0, 0) << ' ';
	std::cout << covMat(0, 1) << std::endl;
	std::cout << covMat(1, 0) << ' ';
	std::cout << covMat(1, 1) << std::endl;
*/

	EigenSolver<MatrixXf> es(covMat);
	/*
	cout << "Eigenvalue:"
		<< endl << es.eigenvalues() << endl;

	cout << "Eigenvectors:"
		<< endl << es.eigenvectors() << endl;

	*/
	float eigenvalueCalc[2] = { es.eigenvalues()[0].real(), es.eigenvalues()[1].real() };

	float eigenvectorCalc[2][2] = { { es.eigenvectors()(0, 0).real(), es.eigenvectors()(0, 1).real() }, { es.eigenvectors()(1, 0).real(), es.eigenvectors()(1, 1).real() } };

	int index = -1;

	if (eigenvalueCalc[0] > eigenvalueCalc[1])
	{
		index = 0;
	}
	else
	{
		index = 1;
	}

	float maxEigenVec[2] = { eigenvectorCalc[0][index], eigenvectorCalc[1][index] };
	/*
	std::cout << maxEigenVec[0] << ' ';
	std::cout << maxEigenVec[1] << std::endl;
	*/
	float bladeRotAngle = atan2(maxEigenVec[0], maxEigenVec[1])*(180 / PI);

	if (bladeRotAngle < 0)
	{
		bladeRotAngle = bladeRotAngle + 180;
	}

	

	return bladeRotAngle;
	
}


void lidarData_thresholdByDistance(vector<int>& angles, vector<int>& distances, float minDist, float maxDist){

	std::vector<size_t> results;

	auto it = std::find_if(std::begin(distances), std::end(distances), [maxDist, minDist](int i){return i > maxDist || i < minDist; });
	while (it != std::end(distances)) {
		results.emplace_back(std::distance(std::begin(distances), it));
		it = std::find_if(std::next(it), std::end(distances), [maxDist, minDist](int i){return i > maxDist || i < minDist; });

	}





	for (size_t i = 0; i < results.size(); i++)
	{

		distances[results[i]] = -1;
		angles[results[i]] = -1;
	}

	distances.erase(remove(distances.begin(), distances.end(), -1), distances.end());
	angles.erase(remove(angles.begin(), angles.end(), -1), angles.end());



}


tuple<double, double> calculateMeans(vector<int>& angles, vector<int>& distances){

	double meanDist = -1;
	double meanAngle = -1;

	meanDist = accumulate(distances.begin(), distances.end(), 0.0) / distances.size();


	vector<double> sinAngles;
	vector<double> cosAngles;



	for (size_t i = 0; i < angles.size(); i++)
	{
		sinAngles.push_back(sin(angles[i] * (PI / 180)));
		cosAngles.push_back(cos(angles[i] * (PI / 180)));

	}

	double avgSin = accumulate(sinAngles.begin(), sinAngles.end(), 0.0) / sinAngles.size();

	double avgCos = accumulate(cosAngles.begin(), cosAngles.end(), 0.0) / cosAngles.size();




	if (avgSin>0 && avgCos>0)
	{
		meanAngle = atan(avgSin / avgCos);
	}
	else if (avgCos <0)
	{
		meanAngle = atan(avgSin / avgCos) + 180 * (PI / 180);
	}
	else if (avgSin <0 && avgCos >0)
	{
		meanAngle = atan(avgSin / avgCos) + 360 * (PI / 180);
	}


	meanAngle = meanAngle * (180 / PI);

	return make_tuple(meanAngle, meanDist);

}


void lidar_PolarToCart(double centerX, double centerY, vector<int>& angles, vector<int>& distances, vector<double>& posX, vector<double>& posY){

	for (size_t i = 0; i < distances.size(); i++)
	{
		posX.push_back(centerX + distances[i] * sin(angles[i] * (PI / 180)));
		posY.push_back(centerY + distances[i] * cos(angles[i] * (PI / 180)));
	}

}


tuple<double, double, double, int, int> getPathCenterFromMaxDist(vector<double>& x_pos, vector<double>& y_pos){

	double maxDistance = 0;
	double maxDistance_ind[2] = {-1,-1};

	double tempDist = 0;
	for (size_t i = 0; i < x_pos.size(); i++)
	{
		for (size_t j = 0; j < x_pos.size(); j++)
		{
			tempDist = sqrt(pow(x_pos[i] - x_pos[j], 2) + pow(y_pos[i] - y_pos[j], 2));
			if (tempDist > maxDistance)
			{
				maxDistance = tempDist;
				maxDistance_ind[0] = i;
				maxDistance_ind[1] = j;
			}

		}
	}

	int path_centerIndex = max(maxDistance_ind[0], maxDistance_ind[1]);
	double path_centerX = x_pos[path_centerIndex];
	double path_centerY = y_pos[path_centerIndex];

	return  make_tuple(maxDistance, path_centerX, path_centerY, maxDistance_ind[0], maxDistance_ind[1]);

}



