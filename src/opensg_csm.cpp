#include <cstdlib>
#include <iostream>
#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGMultiDisplayWindow.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OSGCSM/OSGCAVESceneManager.h>
#include <OSGCSM/OSGCAVEConfig.h>
#include <OSGCSM/appctrl.h>
#include <OpenSG/OSGPointLight.h>

OSG_USING_NAMESPACE

OSGCSM::CAVESceneManager *mgr = NULL;
NodeRecPtr scene;

void cleanup()
{
	scene = NULL;
	delete mgr;
}

NodeTransitPtr buildScene()
{
	return makeTorus(25, 100, 32, 64);
}

void reshape(int w, int h)
{
	mgr->resize(w, h);
	glutPostRedisplay();
}

void display(void)
{
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
		case 'e':
			ed = mgr->getEyeSeparation() * .9f;
			std::cout << "Eye distance: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'E':
			ed = mgr->getEyeSeparation() * 1.1f;
			std::cout << "Eye distance: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		default:
			std::cout << "Key '" << k << "' ignored\n";
	}
}

int setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE );
	const int winid = glutCreateWindow("OpenSG CSMDemo");
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
		OSGCSM::CAVEConfig *cfg = NULL;

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
					cfg=new OSGCSM::CAVEConfig();
					if (!cfg->loadFile(cfgFile)) 
					{
						std::cout << "ERROR: could not load config file '" << cfgFile << "'\n";
						return EXIT_FAILURE;
					}
				}
			} else {
				std::cout << "Loading scene file '" << argv[a] << "'\n";
				scene = SceneFileHandler::the()->read(argv[a], NULL);
			}
		}

		if (!cfg) 
		{
			cfg=new OSGCSM::CAVEConfig();
			const char* const default_config_filename = "config/mono.csm";
			if ( ! cfg->loadFile(default_config_filename) ) 
			{
				std::cout << "ERROR: could not load default config file '" << default_config_filename << "'\n";
				return EXIT_FAILURE;
			}
		}

		cfg->printConfig();

		// start servers for video rendering
		if ( startServers(*cfg) < 0 ) 
		{
			std::cout << "ERROR: Failed to start servers\n";
			return EXIT_FAILURE;
		}

		setupGLUT(&argc, argv);

		MultiDisplayWindowRefPtr mwin = createAppWindow(*cfg, cfg->getBroadcastaddress());

		if (!scene)
			scene = buildScene();

		commitChanges();

		mgr = new OSGCSM::CAVESceneManager(cfg);
		mgr->setWindow(mwin );
		mgr->setRoot  (scene);

		// set an initial user position (in case no tracker being used)
		mgr->setUserTransform(Vec3f(0,0,0), Quaternion(Vec3f(0,1,0), 3.141) );
		mgr->setTranslation(Vec3f(0,0,200));
		mgr->showAll();
		mgr->getWindow()->init();
		mgr->turnWandOff();
	}
	glutMainLoop();
}
