#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>
#include <vector>

#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGMultiDisplayWindow.h>
#include <OpenSG/OSGSceneFileHandler.h>

#include <OSGCSM/OSGCAVESceneManager.h>
#include <OSGCSM/OSGCAVEConfig.h>

#include <OSGCSM/appctrl.h>
#include <trackdAPI_CC.h>

OSG_USING_NAMESPACE

OSGCSM::CAVESceneManager *mgr = NULL;
OSGCSM::CAVEConfig cfg;
NodeRefPtr scene = NULL;
TrackerReader* tracker = NULL;
ControllerReader* controller = NULL;
float lastLR=0, lastUD=0;
std::vector<int> button_values;

void cleanup()
{
	scene = NULL;
	delete mgr;
	delete tracker;
	delete controller;
}


NodeTransitPtr buildScene()
{
	return makeTorus(25, 100, 32, 64);
}

void InitTracker(OSGCSM::CAVEConfig &cfg)
{
	try
	{
		const int trackerKey = cfg.getTrackerKey();
		if (trackerKey)
		{
			std::cout << "TrackerReader with key " << trackerKey << '\n';
			tracker = new TrackerReader(trackerKey);
		}

		const int controllerKey = cfg.getControllerKey();
		if (controllerKey) 
		{
			std::cout << "ControllerReader with key " << controllerKey << '\n';
			controller = new ControllerReader(controllerKey);
		}
	}
	catch(const TrackdError& te) 
	{
		std::cout << "ERROR: " << te.what() << '\n';
		return;
	}
}

void printControllerStats(int i)
{
	if (!tracker)
	{
		std::cout << "No tracker\n";
		return;
	}

	std::cout.precision(5);
	std::cout << std::showpoint << "Sensor " << i;
	float vec[3];
	tracker->trackdGetPosition(i, vec);
	std::cout << " position(" << vec[0] << ", " << vec[1] << ", " << vec[2] << ") ";
	tracker->trackdGetEulerAngles(i, vec);
	std::cout << " euler angles(" << vec[0] << ", " << vec[1] << ", " << vec[2] << ")\n";
}

void ButtonChanged(int button, int newState)
{
	if (button == 0)
	{
		if (newState==1 && tracker) 
		{
			const int numSensors = tracker->trackdGetNumberOfSensors();

			for (int i=0; i < numSensors; ++i)
				printControllerStats(i);
		}
	}
}

void CheckTracker()
{
	/* If connected to a shared mem segment for a controller/wand */
	if (controller) 
	{
		const float scale_valuator_left_right = 10.f;
		const float scale_valuator_front_back = 10.f;
		const float ValLR = controller->trackdGetValuator(0) * scale_valuator_left_right; // left-right [-scale_valuator_left_right; scale_valuator_left_right]
		const float ValFB = controller->trackdGetValuator(1) * scale_valuator_front_back; // front-back [-scale_valuator_front_back; scale_valuator_front_back]
		Vec3f transChg = Vec3f(0,0,0);
		float rotateAngle = 0;

		if (fabsf(ValLR) > 0.001) 
		{
			transChg[0] += ValLR;
		}

		if (fabsf(ValFB) > 0.001) 
		{
			// also check if we need to go up or down
			float orn[3];
			tracker->trackdGetEulerAngles(1,orn);
			float b = cosf((orn[0]/180)*Pi);
			transChg[2] -= (b*(ValFB));
			float a= sinf((orn[0]/180)*Pi);
			transChg[1] += (a*(ValFB));
			// set the rotation based on the y axis rotation of wand
			rotateAngle = (orn[1]/180)*Pi;
		}

		// create a matrix of out of the transformation
		Matrix transMtx,rotateMtx;
		transMtx.setIdentity();
		transMtx.setTranslate(transChg);
		rotateMtx.setIdentity();
		rotateMtx.setRotate(Quaternion(Vec3f(0,1,0),rotateAngle));

		// get the managers translation, multiply it by the change and put it back
		Matrix mgrMtx;
		mgrMtx.setTransform(mgr->getTranslation());
		mgrMtx.mult(rotateMtx);
		mgrMtx.mult(transMtx);

		Quaternion Q, Q1;
		Vec3f T, T1;
		mgrMtx.getTransform(T, Q, T1, Q1);

		mgr->setTranslation(T);

		if(cfg.getFollowHead()) 
		{
			float pos[3], orn[3];
			const auto sensor_id_head = cfg.getSensorIDHead();
			tracker->trackdGetPosition(sensor_id_head, pos);
			tracker->trackdGetEulerAngles(sensor_id_head, orn);

			// set up a quaternion for orientation diferences in the head
			Quaternion Q = Quaternion(Vec3f(0,0,0),1);
			Q *= Quaternion(Vec3f(1,0,0),(orn[0]*Pi)/180);
			Q *= Quaternion(Vec3f(0,1,0),(orn[1]*Pi)/180);
			Q *= Quaternion(Vec3f(0,0,1),(orn[2]*Pi)/180);

			// adjusting the translation by 30 for feet to cm conversion, really
			// should be handled through CSM config....
			const float scale_feet2cm = 30.48f;
			mgr->setUserTransform(Vec3f(pos[0]*scale_feet2cm, pos[1]*scale_feet2cm, pos[2]*scale_feet2cm), Q); 
		}

		button_values.resize(controller->trackdGetNumberOfButtons());
		for(std::size_t i = 0; i < button_values.size(); ++i)
		{
			auto buttVal = controller->trackdGetButton(i);
			if (buttVal != button_values[i]) 
			{
				ButtonChanged(i, buttVal);
				button_values[i] = buttVal;
			}
		}
	}
}

void reshape(int w, int h)
{
	mgr->resize(w, h);
	glutPostRedisplay();
}

void display(void)
{
	CheckTracker();
	commitChanges();
	mgr->redraw();

	//the changelist should be cleared - else things
	//could be copied multiple times
	OSG::Thread::getCurrentChangeList()->clear();

	// to ensure a black navigation window
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
}

void keyboard(unsigned char k, int x, int y)
{
	Real32 ed;
	switch(k)
	{
		case 'q':
		case 27: 
			cleanup();
			exit(0);
			break;
		case 'z':
			ed = mgr->getEyeSeparation() * .9f;
			std::cout << "ED: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'x':
			ed = mgr->getEyeSeparation() * 1.1f;
			std::cout << "ED: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'h':
			cfg.setFollowHead(!cfg.getFollowHead());
			std::cout << "following head: " << std::boolalpha << cfg.getFollowHead() << '\n';
			break;
		case 'i':
			printControllerStats(cfg.getSensorIDHead());
			break;
		default:
			std::cout << "Key '" << k << "' ignored\n";
	}
}

int setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE );
	const int winid = glutCreateWindow("OpenSG CSMDemo with TrackdAPI");
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(display);
	return winid;
}

int main(int argc, char **argv)
{
#if WIN32
	OSG::preloadSharedObject("OSGFileIO");
	OSG::preloadSharedObject("OSGImageFileIO");
#endif
	{
		bool cfgIsSet = false;

		// ChangeList needs to be set for OpenSG 1.4
		ChangeList::setReadWriteDefault();
		osgInit(argc,argv);

		// evaluate intial params
		for(int a=1 ; a<argc ; ++a)
		{
			if( argv[a][0] == '-' )
			{
				if ( strcmp(argv[a],"-f") == 0 ) 
				{
					char* cfgFile = argv[a][2] ? &argv[a][2] : &argv[++a][0];
					if (!cfg.loadFile(cfgFile)) 
					{
						std::cout << "ERROR: could not load config file '" << cfgFile << "'\n";
						return EXIT_FAILURE;
					}
					cfgIsSet = true;
				}
			} else {
				std::cout << "Loading scene file '" << argv[a] << "'\n";
				scene = SceneFileHandler::the()->read(argv[a], NULL);
			}
		}

		// load the CAVE setup config file if it was not loaded already:
		if ( ! cfgIsSet ) 
		{
			const char* const default_config_filename = "config/mono.csm";
			if ( ! cfg.loadFile(default_config_filename) ) 
			{
				std::cout << "ERROR: could not load default config file '" << default_config_filename << "'\n";
				return EXIT_FAILURE;
			}
		}

		cfg.printConfig();

		// start servers for video rendering
		if ( startServers(cfg) < 0 ) 
		{
			std::cout << "ERROR: Failed to start servers\n";
			return EXIT_FAILURE;
		}

		setupGLUT(&argc, argv);

		InitTracker(cfg);

		// MultiDisplayWindowRefPtr mwin = createAppWindow(cfg,"10.0.255.255");
		MultiDisplayWindowRefPtr mwin = createAppWindow(cfg, cfg.getBroadcastaddress());

		if (!scene) 
			scene = buildScene();

		commitChanges();
		// create the CAVESceneManager helper + tell it what to manage
		mgr = new OSGCSM::CAVESceneManager(&cfg);
		mgr->setWindow(mwin );
		mgr->setRoot  (scene);

		// set an initial user position (in case no tracker being used)
		mgr->setUserTransform(Vec3f(0,0,0), Quaternion(Vec3f(0,0,0),0) );
		mgr->setTranslation(Vec3f(0,0,200));
		mgr->showAll();
		mgr->getWindow()->init();
		mgr->turnWandOff();
	}
	glutMainLoop();
}
