/*
 * ConfigFileLoader.h
 *
 *  Created on: Jul 7, 2016
 *      Author: farbod
 */

#ifndef CONFIGFILELOADER_H_
#define CONFIGFILELOADER_H_

#include <Eigen/Dense>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>


/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
template <typename Derived>
void loadMatrix(const std::string& filename,
		const std::string& matrixName, Eigen::MatrixBase<Derived>& matrix,
		bool print=false)
{
	size_t rows = matrix.rows();
	size_t cols = matrix.cols();

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	double scaling = pt.get<double>(matrixName + ".scaling", 1);

	for (size_t i=0; i<rows; i++)
	{
		for (size_t j=0; j<cols; j++)
		{
			matrix(i,j) = scaling*pt.get<double>(matrixName + "." + "(" +std::to_string(i) + "," + std::to_string(j) + ")" , 0.0);
		}
	}

	if (print==true) {
		std::cout << matrixName << ": \n" <<
				"====================================================================="
				<< std::endl;
		if (cols==1)
			std::cout << matrix.transpose() << std::endl;
		else if (rows==1)
			std::cout << matrix << std::endl;
		else
			std::cout << matrix << std::endl;

		std::cout << std::endl;
	}
}


/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void loadSimulationSettings(const std::string& filename, double& dt,
		double& tFinal, double& initSettlingTime)
{
	const double defaultInitSettlingTime = 1.5;
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	dt     = pt.get<double>("simulationSettings.dt");
	tFinal = pt.get<double>("simulationSettings.tFinal");
	initSettlingTime = pt.get<double>("simulationSettings.initSettlingTime", defaultInitSettlingTime);

	std::cerr<<"Simulation Settings: " << std::endl;
	std::cerr<<"=====================================" << std::endl;
	std::cerr<<"Simulation time step ......... " << dt << std::endl;
	std::cerr<<"Controller simulation time ... [0, " << tFinal  << "]" << std::endl;
	std::cerr<<"initial settling time ......... " << initSettlingTime  << std::endl << std::endl;

}



/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void loadVisualizationSettings(const std::string& filename,
		double& slowdown, double& vizTime)
{
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	slowdown = pt.get<double>("visualization.slowdown");
	vizTime = pt.get<double>("visualization.vizTime");
}

#endif /* CONFIGFILELOADER_H_ */
