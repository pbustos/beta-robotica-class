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


/** \mainpage RoboComp::task1
 *
 * \section intro_sec Introduction
 *
 * The task1 component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd task1
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/task1 --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <clocale>
#include <cstdlib>   // std::_Exit -- see the exit note at the end of main()
#include <cstdio>    // std::fflush
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtWidgets>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <ConfigLoader/ConfigLoader.h>

#include <sigwatch/sigwatch.h>

#include "genericworker.h"
#include "../src/specificworker.h"


#include <Camera360RGB.h>
#include <GenericBase.h>
#include <Lidar3D.h>
#include <OmniRobot.h>
#include <Person.h>
#include <VisualElements.h>

#define USE_QTGUI

#define PROGRAM_NAME    "task1"
#define SERVER_FULL_NAME   "RoboComp task1::task1"


template <typename ProxyType, typename ProxyPointer>
void require(const Ice::CommunicatorPtr& communicator,
             const std::string& proxyConfig, 
             const std::string& proxyName,
             ProxyPointer& proxy)
{
    try
    {
        proxy = Ice::uncheckedCast<ProxyType>(communicator->stringToProxy(proxyConfig));
        std::cout << proxyName << " initialized Ok!\n";
    }
    catch(const Ice::Exception& ex)
    {
        std::cout << "[" << PROGRAM_NAME << "]: Exception creating proxy " << proxyName << ": " << ex;
        throw;
    }
}


class task1 : public Ice::Application
{
public:
	task1 (QString configFile, QString prfx, bool startup_check) { 
		this->configFile = configFile.toStdString();
		this->prefix = prfx.toStdString();
		this->startup_check_flag=startup_check; 

		initialize();
		}

	Ice::InitializationData getInitializationDataIce();

private:
	void initialize();
	std::string prefix, configFile;
	ConfigLoader configLoader;
	TuplePrx tprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

Ice::InitializationData task1::getInitializationDataIce(){
        Ice::InitializationData initData;
        initData.properties = Ice::createProperties();

        // Forward EVERY Ice property the config declares, not a fixed list of four.
        //
        // Until 2026-09-11 this function copied exactly Ice.Warn.Connections, Ice.Trace.Network,
        // Ice.Trace.Protocol and Ice.MessageSizeMax, and nothing else. Every other Ice setting a
        // component config declared was therefore INERT: it was loaded, printed by printConfig(),
        // and never reached the communicator. A config block that is read and ignored looks exactly
        // like one that works, so this cost real time to find — in webots-bridge the config had
        // carried an [omnirobot.ThreadPool] block for months, with a comment explaining that it
        // gave base commands their own dispatch thread so they could not queue behind a camera.
        // They had always shared the default pool. It only became visible when some servants began
        // to block: four blocking calls took the single pool and the whole fleet serialised, an
        // untouched IMU stream falling from 113 Hz to 29 Hz while its data stayed 5 ms fresh.
        //
        // These properties are only read when the communicator is created, which is why this must
        // happen here and cannot be moved into SpecificWorker.
        //
        // "Ice.*" covers the run time's own settings; "*.ThreadPool.*" covers the per-object-adapter
        // pools, which are named after the adapter (see the implement<>() calls below) and so cannot
        // be listed in advance. Anything else in the config is the component's own business and is
        // deliberately not forwarded. A value that will not convert to a string is skipped rather
        // than aborting startup.
        for (const auto &key_view : this->configLoader.getKeys())
        {
            const std::string key{key_view};
            const bool is_ice_runtime  = key.rfind("Ice.", 0) == 0;
            const bool is_adapter_pool = key.find(".ThreadPool.") != std::string::npos;
            if (not (is_ice_runtime or is_adapter_pool))
                continue;
            try { initData.properties->setProperty(key, this->configLoader.get<std::string>(key)); }
            catch (...) { /* not string-convertible: leave Ice's default for this one */ }
        }
		return initData;
}

void task1::initialize()
{
    this->configLoader.load(this->configFile);
	this->configLoader.printConfig();
	std::cout<<std::endl;
}

int task1::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif

	// The Q*Application constructor above calls setlocale(LC_ALL, "") — it adopts the user's
	// locale for the C library. On a machine whose locale uses a decimal COMMA (es_ES, fr_FR,
	// de_DE, ...) that silently breaks numeric I/O: strtof/atof/scanf then stop at the '.' of a
	// file written with decimal POINTS and return just the integer part, with NO error flag
	// ("0.626452" -> 0). Meanwhile std::ofstream keeps formatting through the C++ global locale,
	// which stays "C" and writes POINTS — so a component corrupts its own data on round-trip.
	// Pin LC_NUMERIC back to "C" so the two agree. Text stays localised: LC_CTYPE / LC_TIME /
	// LC_MESSAGES keep the user's locale, and Qt's own formatting goes through QLocale, which
	// this does not touch. Prefer std::from_chars for parsing anyway — it is locale-independent
	// by definition and cannot regress if someone changes the locale later.
	std::setlocale(LC_NUMERIC, "C");

	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

	RoboCompCamera360RGB::Camera360RGBPrxPtr camera360rgb_proxy;
	RoboCompLidar3D::Lidar3DPrxPtr lidar3d_proxy;
	RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy;
	RoboCompVisualElements::VisualElementsPrxPtr visualelements_proxy;


	//Require code
	require<RoboCompCamera360RGB::Camera360RGBPrx, RoboCompCamera360RGB::Camera360RGBPrxPtr>(communicator(),
	                    configLoader.get<std::string>("Proxies.Camera360RGB"), "Camera360RGBProxy", camera360rgb_proxy);
	require<RoboCompLidar3D::Lidar3DPrx, RoboCompLidar3D::Lidar3DPrxPtr>(communicator(),
	                    configLoader.get<std::string>("Proxies.Lidar3D"), "Lidar3DProxy", lidar3d_proxy);
	require<RoboCompOmniRobot::OmniRobotPrx, RoboCompOmniRobot::OmniRobotPrxPtr>(communicator(),
	                    configLoader.get<std::string>("Proxies.OmniRobot"), "OmniRobotProxy", omnirobot_proxy);
	require<RoboCompVisualElements::VisualElementsPrx, RoboCompVisualElements::VisualElementsPrxPtr>(communicator(),
	                    configLoader.get<std::string>("Proxies.VisualElements"), "VisualElementsProxy", visualelements_proxy);

	tprx = std::make_tuple(camera360rgb_proxy,lidar3d_proxy,omnirobot_proxy,visualelements_proxy);
	SpecificWorker *worker = new SpecificWorker(this->configLoader, tprx, startup_check_flag);
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));

	try
	{

		// Server adapter creation and publication
		std::cout << SERVER_FULL_NAME " started" << std::endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		std::cerr << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << std::endl;
		std::cerr << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	delete worker;
	return status;
}

int main(int argc, char* argv[])
{
	std::string arg;

	// Set config file
	QString configFile("etc/config");
	bool startup_check_flag = false;
	QString prefix("");
	if (argc > 1)
	{

		// Search in argument list for arguments
		QString startup = QString("--startup-check");
		QString initIC = QString("--Ice.Config=");
		QString prfx = QString("--prefix=");
		for (int i = 0; i < argc; ++i)
		{
			arg = argv[i];
			if (arg.find(startup.toStdString(), 0) != std::string::npos)
			{
				startup_check_flag = true;
				std::cout << "Startup check = True"<< std::endl;
			}
			else if (arg.find(prfx.toStdString(), 0) != std::string::npos)
			{
				prefix = QString::fromStdString(arg).remove(0, prfx.size());
				if (prefix.size()>0)
					prefix += QString(".");
				printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
			}
			else if (arg.find(initIC.toStdString(), 0) != std::string::npos)
			{
				configFile = QString::fromStdString(arg).remove(0, initIC.size());
				qDebug()<<__LINE__<<"Starting with config file:"<<configFile;
			}
			else if (i==1 and argc==2 and arg.find("--", 0) == std::string::npos)
			{
				configFile = QString::fromStdString(arg);
				qDebug()<<__LINE__<<QString::fromStdString(arg)<<argc<<arg.find("--", 0)<<"Starting with config file:"<<configFile;
			}
		}

	}
	task1 app(configFile, prefix, startup_check_flag);

	const int status = app.main(argc, argv, app.getInitializationDataIce());

	// Leave the process HERE, before C++ static destruction and the dynamic linker's finalizers.
	//
	// libIce 3.7 destroys its own globals in the wrong order. At exit _dl_call_fini runs libIce's
	// finalizer, a static object inside it calls
	// IceInternal::FactoryTable::removeExceptionFactory(), and that locks a mutex an EARLIER
	// finalizer has already destroyed. pthread_mutex_lock returns EINVAL, IceUtil::Mutex::lock()
	// (IceUtil/Mutex.h:295) throws ThreadSyscallException out of a destructor, and the process
	// dies on EVERY otherwise-clean shutdown with
	//     terminate called after throwing an instance of 'IceUtil::ThreadSyscallException'
	//     what(): include/IceUtil/Mutex.h:295: syscall exception: Invalid argument
	// and SIGABRT (exit 134) plus a core dump. The whole fault is inside libIce's own finalizer
	// chain, so no link order or teardown ordering on the component's side can avoid it.
	//
	// Nothing the component owns is skipped: run() above ends with `delete worker`, so
	// SpecificWorker and everything it holds -- DSR graphs, their FastDDS DomainParticipants, the
	// owned-node cleanup in the destructor -- have already run, and Qt's application object is
	// gone too. Only third-party library static teardown is bypassed. Verified against a DSR
	// agent holding two DomainParticipants: no /dev/shm FastDDS segments leaked (99 before, 99
	// after), clean exit code 0 on both SIGTERM and SIGINT, no core dump.
	//
	// std::_Exit does not flush stdio, so flush first or the tail of the log is lost.
	std::cout.flush();
	std::cerr.flush();
	std::fflush(nullptr);
	std::_Exit(status);
}
