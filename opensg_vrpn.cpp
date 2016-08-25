#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>

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
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGSimpleTexturedMaterial.h>

#include <vrpn_Tracker.h>
#include <vrpn_Button.h>
#include <vrpn_Analog.h>

OSG_USING_NAMESPACE

OSGCSM::CAVEConfig cfg;
OSGCSM::CAVESceneManager *mgr = nullptr;
vrpn_Tracker_Remote* tracker =  nullptr;
vrpn_Button_Remote* button = nullptr;
vrpn_Analog_Remote* analog = nullptr;

void cleanup()
{
	delete mgr;
	delete tracker;
	delete button;
	delete analog;
}

void print_tracker();

auto head_orientation = Quaternion(Vec3f(0.f, 1.f, 0.f), 3.141f);
auto head_position = Vec3f(0.f, 0.f, 0.f);	// a 1.7m Person 2m in front of the scene

auto wand_orientation = Quaternion();
auto wand_position = Vec3f();

void attachGroundToSceneGraph(NodeRecPtr root)
{
	NodeRecPtr ground = Node::create();

	/* create and attach the ground */
	NodeRecPtr geo = makePlane(270, 270, 100, 100);

	ComponentTransformRecPtr ct = ComponentTransform::create();
	ct->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(90)));
	ct->setTranslation(Vec3f(0,0,0));

	SimpleMaterialRecPtr material = SimpleMaterial::create();
	material->setDiffuse(Color3f(1,0.8f,0));
	material->setAmbient(Color3f(0.8f, 0.2f, 0.2f));

	MaterialGroupRecPtr materialGroup = MaterialGroup::create();
	materialGroup->setMaterial(material);
	
	NodeRecPtr transformations = Node::create();

	transformations->setCore(ct);
	transformations->addChild(geo);
	ground->setCore(materialGroup);
	ground->addChild(transformations);

	root->addChild(ground);
}

void attachSideWall(NodeRecPtr root, Vec3f translation)
{
	NodeRecPtr plane = Node::create();

	/* create and attach the ground */
	NodeRecPtr geo = makePlane(1000, 100, 50, 50);

	ComponentTransformRecPtr ct = ComponentTransform::create();
	ct->setTranslation(translation);
	ct->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(90)));

	SimpleMaterialRecPtr material = SimpleMaterial::create();
	material->setDiffuse(Color3f(0,0.8f,1));
	material->setAmbient(Color3f(0.2f, 0.8f, 0.2f));

	MaterialGroupRecPtr materialGroup = MaterialGroup::create();
	materialGroup->setMaterial(material);
	
	NodeRecPtr transformations = Node::create();

	transformations->setCore(ct);
	transformations->addChild(geo);
	plane->setCore(materialGroup);
	plane->addChild(transformations);

	// put the nodes in the scene again
	root->addChild(plane);
}

void attachSunToSceneGraph(NodeRecPtr root) {
	// Create beacon for sun position
	NodeRecPtr sunChild = Node::create();
	GeometryRecPtr sunGeo = makeSphereGeo(2,10);
	sunChild->setCore(sunGeo);

	TransformRecPtr sunTransCore = Transform::create();
	Matrix sunMatrix;
	sunMatrix.setIdentity();
	sunMatrix.setTranslate(0,200,-200);
	sunTransCore->setMatrix(sunMatrix);

	NodeRecPtr sunTrans = makeNodeFor(sunTransCore);
	sunTrans->addChild(sunChild);
	
	PointLightRecPtr sunLight = PointLight::create();
	//sunLight->setAttenuation(1,0,2);

	//color informationa
	sunLight->setDiffuse(Color4f(1,1,1,1));
	sunLight->setAmbient(Color4f(0.2f,0.2f,0.2f,1));
	sunLight->setSpecular(Color4f(1,1,1,1));

	sunLight->setBeacon(sunTrans);
	root->addChild(sunTrans);
	root->setCore(sunLight);

	//DirectionalLightRecPtr dirLight = DirectionalLight::create();
	//dirLight->setDirection(1,1,-1);

	//color information
	//dirLight->setDiffuse(Color4f(1,1,1,1));
	//dirLight->setAmbient(Color4f(0.2f,0.2f,0.2f,1));
	//dirLight->setSpecular(Color4f(1,1,1,1));

	//wrap the root, cause only nodes below the lights will be lit
	//return makeNodeFor(dirLight);
}

void attachTorus(NodeRecPtr root)
{
	NodeRecPtr plane = Node::create();

	/* create and attach the ground */
	NodeRecPtr geo = makeTorus(10.f, 50.f, 32.f, 64.f);

	ComponentTransformRecPtr ct = ComponentTransform::create();
	ct->setTranslation(Vec3f(0.f,0.f,-500.f));

	SimpleMaterialRecPtr material = SimpleMaterial::create();
	material->setDiffuse(Color3f(0,0.8f,1));
	material->setAmbient(Color3f(0.2f, 0.8f, 0.2f));

	MaterialGroupRecPtr materialGroup = MaterialGroup::create();
	materialGroup->setMaterial(material);
	
	NodeRecPtr transformations = Node::create();

	transformations->setCore(ct);
	transformations->addChild(geo);
	plane->setCore(materialGroup);
	plane->addChild(transformations);

	// put the nodes in the scene again
	root->addChild(plane);
}

NodeTransitPtr createScenegraph() {
	NodeRecPtr root = Node::create();
	attachSunToSceneGraph(root);
	
	NodeRecPtr scene = Node::create();
	
	ComponentTransformRecPtr ct = ComponentTransform::create();
	scene->setCore(ct);
	root->addChild(scene);

	attachGroundToSceneGraph(scene);
	attachSideWall(scene, Vec3f(-136,50,0));
	attachSideWall(scene, Vec3f(136,50,0));
	attachTorus(scene);

	return NodeTransitPtr(root);
}

template<typename T>
T scale_tracker2cm(const T& value)
{
	static const float scale = OSGCSM::convert_length(cfg.getUnits(), 1.f, OSGCSM::CAVEConfig::CAVEUnitCentimeters);
	return value * scale;
}

void VRPN_CALLBACK callback_head_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	head_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	head_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

void VRPN_CALLBACK callback_wand_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	wand_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	wand_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

auto analog_values = Vec3f();
void VRPN_CALLBACK callback_analog(void* userData, const vrpn_ANALOGCB analog)
{
	if (analog.num_channel >= 2)
		analog_values = Vec3f(analog.channel[0], 0, -analog.channel[1]);
}

void VRPN_CALLBACK callback_button(void* userData, const vrpn_BUTTONCB button)
{
	if (button.button == 0 && button.state == 1)
		print_tracker();
}

void InitTracker(OSGCSM::CAVEConfig &cfg)
{
	try
	{
		const char* const vrpn_name = "DTrack@localhost";
		tracker = new vrpn_Tracker_Remote(vrpn_name);
		tracker->shutup = true;
		tracker->register_change_handler(NULL, callback_head_tracker, cfg.getSensorIDHead());
		tracker->register_change_handler(NULL, callback_wand_tracker, cfg.getSensorIDController());
		button = new vrpn_Button_Remote(vrpn_name);
		button->shutup = true;
		button->register_change_handler(nullptr, callback_button);
		analog = new vrpn_Analog_Remote(vrpn_name);
		analog->shutup = true;
		analog->register_change_handler(NULL, callback_analog);
	}
	catch(const std::exception& e) 
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return;
	}
}

void check_tracker()
{
	tracker->mainloop();
	button->mainloop();
	analog->mainloop();
}

void print_tracker()
{
	std::cout << "Head position: " << head_position << " orientation: " << head_orientation << '\n';
	std::cout << "Wand position: " << wand_position << " orientation: " << wand_orientation << '\n';
	std::cout << "Analog: " << analog_values << '\n';
}

void keyboard(unsigned char k, int x, int y)
{
	Real32 ed;
	switch(k)
	{
		case 'q':
		case 27: 
			cleanup();
			exit(EXIT_SUCCESS);
			break;
		case 'w':
			if (head_position[1] < 270.f) {
				head_position = head_position + Vec3f(0.f,2.f,0.f);
			}
			break;
		case 's':
			if (head_position[1] > -2.f) {
				head_position = head_position + Vec3f(0.f,-2.f,0.f);
			}
			break;
		case 'r':
			if (head_position[2] > -133.f) {
				head_position = head_position + Vec3f(0.f,0.f,-2.f);
			}
			break;
		case 'f':
			if (head_position[2] < 133.f) {
				head_position = head_position + Vec3f(0.f,0.f,2.f);
			}
			break;
		case 'a':
			if (head_position[0] > -133.f) {
				head_position = head_position + Vec3f(-2.f,0.f,0.f);
			}
			break;
		case 'd':
			if (head_position[0] < 133.f) {
				head_position = head_position + Vec3f(2.f,0.f,0.f);
			}
			break;
		case 'u':
			head_orientation = head_orientation + Quaternion(Vec3f(0.1f, 0.f, 0.f), 3.141f);
			break;
		case 'j':
			head_orientation = head_orientation + Quaternion(Vec3f(-0.1f, 0.f, 0.f), 3.141f);
			break;
		case 'h':
			break;
		case 'k':
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
		case 't':
			cfg.setFollowHead(!cfg.getFollowHead());
			std::cout << "following head: " << std::boolalpha << cfg.getFollowHead() << '\n';
			break;
		case 'i':
			print_tracker();
			break;
		default:
			std::cout << "Key '" << k << "' ignored\n";
	}
}

void setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB  |GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("OpenSG CSMDemo with VRPN API");
	glutDisplayFunc([]()
	{
		// black navigation window
		glClear(GL_COLOR_BUFFER_BIT);
		glutSwapBuffers();
	});
	glutReshapeFunc([](int w, int h)
	{
		mgr->resize(w, h);
		glutPostRedisplay();
	});
	glutKeyboardFunc(keyboard);
	glutIdleFunc([]()
	{
		check_tracker();
		const auto speed = 1.f;
		mgr->setUserTransform(head_position, head_orientation);
		mgr->setTranslation(mgr->getTranslation() + speed * analog_values);
		commitChanges();
		mgr->redraw();
		// the changelist should be cleared - else things could be copied multiple times
		OSG::Thread::getCurrentChangeList()->clear();
	});
}

int main(int argc, char **argv)
{
#if WIN32
	OSG::preloadSharedObject("OSGFileIO");
	OSG::preloadSharedObject("OSGImageFileIO");
#endif
	try
	{
		bool cfgIsSet = false;
		NodeRefPtr scene = nullptr;

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
		if (!cfgIsSet) 
		{
			const char* const default_config_filename = "config/mono.csm";
			if (!cfg.loadFile(default_config_filename)) 
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

		MultiDisplayWindowRefPtr mwin = createAppWindow(cfg, cfg.getBroadcastaddress());

		if (!scene)  {
			scene = createScenegraph();
		}
		commitChanges();

		mgr = new OSGCSM::CAVESceneManager(&cfg);
		mgr->setWindow(mwin );
		mgr->setRoot(scene);
		mgr->showAll();
		mgr->getWindow()->init();
		mgr->turnWandOff();
	}
	catch(const std::exception& e)
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return EXIT_FAILURE;
	}

	glutMainLoop();
}
