/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>
#include <QDateTime>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

	try { robot_def_ = configLoader.get<std::string>("Grader.RobotDEF"); } catch(...) {}
	try { person_def_ = configLoader.get<std::string>("Grader.PersonDEF"); } catch(...) {}
	try { duration_ = configLoader.get<double>("Grader.Duration"); } catch(...) {}
	try { score_ = FollowScore(configLoader.get<double>("Grader.DistanceRef")); } catch(...) {}
	try { results_file_ = configLoader.get<std::string>("Grader.ResultsFile"); } catch(...) {}

	label_dist_name->setText(QString("|distance - %1 m| (mm)").arg(score_.d_ref() / 1000.0));
	connect(pushButton_start, &QPushButton::clicked, this, &SpecificWorker::start_stop);
	plot_distance->set_labels(QString("distance - %1 m").arg(score_.d_ref() / 1000.0), "mm");
	plot_angle->set_labels("person_angle, + = person to the right", "rad");
	plot_distance->set_duration(duration_);
	plot_angle->set_duration(duration_);
	update_display(duration_);

	try { lineEdit_student->setText(QString::fromStdString(configLoader.get<std::string>("Grader.Student"))); } catch(...) {}

	bool auto_start = false;
	try { auto_start = configLoader.get<bool>("Grader.AutoStart"); } catch(...) {}
	if (auto_start) start_stop();
}


void SpecificWorker::compute()
{
	if (not running_) return;
	sample();
	if (remaining() <= 0.0) finish(true);
	else update_display(remaining());
}


void SpecificWorker::start_stop()
{
	if (running_) { finish(false); return; }
	score_.reset();
	frozen_samples_ = 0;
	plot_distance->clear();
	plot_angle->clear();
	running_ = true;
	t0_ = std::chrono::steady_clock::now();
	pushButton_start->setText("Stop");
	lineEdit_student->setEnabled(false);
	label_status->setText("measuring");
	update_display(duration_);
}


void SpecificWorker::sample()
{
	try
	{
		const auto robot = webots2robocomp_proxy->getObjectPose(robot_def_);
		const auto person = webots2robocomp_proxy->getObjectPose(person_def_);
		const auto s = score_.add(robot, person);
		const auto same = [](const RoboCompWebots2Robocomp::ObjectPose& a, const RoboCompWebots2Robocomp::ObjectPose& b)
			{ return a.position.x == b.position.x and a.position.y == b.position.y and a.orientation.z == b.orientation.z; };
		frozen_samples_ = same(robot, last_robot_) and same(person, last_person_) ? frozen_samples_ + 1 : 0;
		last_robot_ = robot; last_person_ = person;
		if (frozen_samples_ == FROZEN_WARN)
			label_status->setText("the poses have not changed for 2 s: is Webots running, and is Proxies.Webots2Robocomp the "
			                      "port of the webots-bridge (10006 default config, 20006 chocachoca_config)?");
		else if (frozen_samples_ == 0 and label_status->text().startsWith("the poses"))
			label_status->setText("measuring");
		label_dist_now->setText(QString::number(s.distance - score_.d_ref(), 'f', 0));
		label_angle_now->setText(QString::number(s.angle, 'f', 3));
		const double t = duration_ - remaining();
		plot_distance->add(t, s.distance - score_.d_ref());
		plot_angle->add(t, s.angle);
	}
	catch (const Ice::Exception& e)
	{
		label_status->setText(QString("bridge error, sample skipped: ") + e.what());
	}
}


double SpecificWorker::remaining() const
{
	return duration_ - std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
}


void SpecificWorker::update_display(double remaining)
{
	const int secs = static_cast<int>(std::ceil(std::max(0.0, remaining)));
	label_time->setText(QString("%1:%2").arg(secs / 60, 2, 10, QChar('0')).arg(secs % 60, 2, 10, QChar('0')));
	label_dist_sum->setText(QString::number(score_.sum_distance_error(), 'f', 0));
	label_dist_mean->setText(QString::number(score_.mean_distance_error(), 'f', 1));
	label_angle_sum->setText(QString::number(score_.sum_angle_error(), 'f', 2));
	label_angle_mean->setText(QString::number(score_.mean_angle_error(), 'f', 4));
	label_dist_max->setText(QString::number(score_.max_distance_error(), 'f', 0));
	label_dist_std->setText(QString::number(score_.std_distance_error(), 'f', 0));
	label_angle_max->setText(QString::number(score_.max_angle_error(), 'f', 3));
	label_angle_std->setText(QString::number(score_.std_angle_error(), 'f', 4));
	label_samples->setText(QString::number(score_.samples()));
}


void SpecificWorker::finish(bool completed)
{
	running_ = false;
	const double elapsed = duration_ - std::max(0.0, remaining());
	update_display(completed ? 0.0 : remaining());
	pushButton_start->setText("Start");
	lineEdit_student->setEnabled(true);

	// one line per measurement; header when the file is new, and upgraded in place if an older version wrote it
	static const std::string header = "date,student,completed,duration_s,samples,sum_dist_err_mm,mean_dist_err_mm,"
	                                  "sum_angle_err_rad,mean_angle_err_rad,max_dist_err_mm,var_dist_err_mm2,"
	                                  "max_angle_err_rad,var_angle_err_rad2,std_dist_err_mm,std_angle_err_rad";
	std::vector<std::string> lines;
	{
		std::ifstream in(results_file_);
		for (std::string l; std::getline(in, l);) lines.push_back(l);
	}
	if (not lines.empty() and lines.front() != header)
	{
		lines.front() = header;
		std::ofstream rewrite(results_file_, std::ios::trunc);
		for (const auto& l : lines) rewrite << l << '\n';
	}
	const bool is_new = lines.empty();
	std::ofstream out(results_file_, std::ios::app);
	if (not out)
	{
		label_status->setText(QString("could not write ") + QString::fromStdString(results_file_));
		return;
	}
	if (is_new)
		out << header << '\n';
	out << QDateTime::currentDateTime().toString(Qt::ISODate).toStdString() << ','
	    << '"' << lineEdit_student->text().toStdString() << '"' << ','
	    << (completed ? 1 : 0) << ',' << std::fixed << std::setprecision(1) << elapsed << ','
	    << score_.samples() << ','
	    << std::setprecision(1) << score_.sum_distance_error() << ',' << score_.mean_distance_error() << ','
	    << std::setprecision(4) << score_.sum_angle_error() << ',' << score_.mean_angle_error() << ','
	    << std::setprecision(1) << score_.max_distance_error() << ',' << score_.var_distance_error() << ','
	    << std::setprecision(4) << score_.max_angle_error() << ',' << score_.var_angle_error() << ','
	    << std::setprecision(1) << score_.std_distance_error() << ','
	    << std::setprecision(4) << score_.std_angle_error() << '\n';
	label_status->setText(QString(completed ? "done" : "aborted") + ", saved to " + QString::fromStdString(results_file_));
}


void SpecificWorker::emergency()
{
    fps.print("Emergency worker", 3000);
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}



/**************************************/
// From the RoboCompWebots2Robocomp you can call this methods:
// RoboCompWebots2Robocomp::ObjectPose this->webots2robocomp_proxy->getObjectPose(string DEF)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->resetWebots()
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setDoorAngle(string DEF, float angle)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setObjectPose(string DEF, ObjectPose pose)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setPathToHuman(int humanId, RoboCompGridder::TPath path)

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion
// RoboCompWebots2Robocomp::ObjectPose

