#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "HW4Functions.h"
#include "hw/HW4.h"
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}
		// Getter methods for private members
    	double getEta() const { return eta; }
    	double getZetta() const { return zetta; }
		double getDStar() const { return d_star; }
		double getQStar() const { return Q_star; }
		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
		MyPotentialFunction(const MyGDAlgorithm& algorithm, const amp::Problem2D& problem) 
        : algorithm(algorithm), problem(problem) {}

		double Attractor(Eigen::Vector2d) const;
		double Repulsor(Eigen::Vector2d) const;
		Eigen::Vector2d Gradient(Eigen::Vector2d q) const;
		Eigen::Vector2d MinimumDistance(Eigen::Vector2d q, const amp::Obstacle2D& obstacle) const;
		Eigen::Vector2d NesterovUpdate(Eigen::Vector2d& q, double learning_rate, double momentum);
		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
<<<<<<< HEAD
			double U_att = Attractor(q);
			double U_rep = Repulsor(q);
			return (U_att + U_rep);
		}
	private: 
	const MyGDAlgorithm& algorithm; 
	amp::Problem2D problem;	
	Eigen::Vector2d velocity;
=======
            return q[0] * q[0] + q[1] * q[1];
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
            return Eigen::Vector2d(q[0] * q[0],  q[1] * q[1]);
        }
>>>>>>> 63d1215bf02fe84e824c8a70813fc39227259163
};