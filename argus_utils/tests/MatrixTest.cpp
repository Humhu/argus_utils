#include "argus_utils/utils/MatrixUtils.h"

using namespace argus;

int main( int argc, char** argv )
{
	MatrixType mat = MatrixType::Random(3,3);
	VectorType lower = GetLowerTriangular( mat, 0 );
	VectorType lower1 = GetLowerTriangular( mat, 1 );

	std::cout << "mat:" << std::endl << mat << std::endl;
	std::cout << "lower: " << lower.transpose() << std::endl;
	std::cout << "lower1: " << lower1.transpose() << std::endl;

	try
	{
		mat = MatrixType::Random(3,4);
		lower = GetLowerTriangular( mat, 0 );
		std::cout << "Failed 3-4 test." << std::endl;
	}
	catch( std::runtime_error e )
	{
		std::cout << "Passed 3-4 test." << std::endl;
	}

	try
	{
		mat = MatrixType::Random(4,3);
		lower = GetLowerTriangular( mat, 0 );
		std::cout << "Failed 4-3 test." << std::endl;
	}
	catch( std::runtime_error e )
	{
		std::cout << "Passed 4-3 test." << std::endl;
	}

	TestPositiveDefinite( mat );
	TestPositiveSemidefinite( mat );

	return 0;
}