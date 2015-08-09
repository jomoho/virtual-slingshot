#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>
#include <chrono>
#include <ctime>
#include <algorithm>

#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGMultiDisplayWindow.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGSimpleSceneManager.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGComponentTransform.h>
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGImage.h>
#include <OpenSG/OSGSimpleTexturedMaterial.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGTextureBackground.h>
#include <OpenSG/OSGGradientBackground.h>
#include <OpenSG/OSGPointLight.h>
#include <OpenSG/OSGSpotLight.h>

#include <OSGCSM/OSGCAVESceneManager.h>
#include <OSGCSM/OSGCAVEConfig.h>
#include <OSGCSM/appctrl.h>

#include <vrpn_Tracker.h>
#include <vrpn_Button.h>
#include <vrpn_Analog.h>



OSG_USING_NAMESPACE

#define HELPERS_ON 0

OSGCSM::CAVEConfig cfg;
OSGCSM::CAVESceneManager *mgr = nullptr;
vrpn_Tracker_Remote* tracker =  nullptr;
vrpn_Button_Remote* button = nullptr;
vrpn_Analog_Remote* analog = nullptr;
auto analog_values = Vec3f();

NodeRecPtr root, 
		landTrans,
		standTrans,
		targetTrans,
		slingTrans,
		projectileTrans,
		pocketTrans,
		helpTrans,
		rollTrans,
		markerTrans,
		ttrans;

NodeRecPtr stringTrans[2],
			helperTrans[4]; 

NodeRecPtr beachTrans;

auto headRot = Quaternion(Vec3f(0.f, 1.f, 0.f), 3.141f);
auto headPos = Vec3f(0.f, 170.f, 200.f);	// a 1.7m Person 2m in front of the scene

auto slingRot = Quaternion();
auto slingPos =  Vec3f(-12, 150, 0);

auto handRot = Quaternion();
auto handPos =  Vec3f(0, 150, 0 );

Vec3f projSpeed, projPos, targetPos = Vec3f(0, 160, -200);

#define SL_AIM 2
#define SL_LOOSE 3
#define SL_LEFT 0
#define SL_RIGHT 1

const Vec3f slingAimOff = Vec3f(0, 10, 0);
const Vec3f slingLeftOff = Vec3f(-6.7f,0,0.5);
const Vec3f slingRightOff = Vec3f(6.7f,0,0.5);
const Vec3f slingLooseOff = Vec3f(0,0,6);
Vec3f slingPoints[4] = {
	slingPos + slingAimOff + slingLeftOff,
	slingPos + slingAimOff + slingRightOff,
	slingPos + slingAimOff,
	slingPos + slingAimOff + slingLooseOff
};

Vec3f stringFollow = slingPoints[SL_LOOSE], stringFollowSpeed = Vec3f(0,0,0);

const Vec3f pocketOff[2] = {Vec3f(-2.6,0,-1.4), Vec3f(2.6,0,-1.4)};
Vec3f pocketPoints[2] ={stringFollow + pocketOff[0], stringFollow + pocketOff[1]};

Quaternion stringRot[2]= {Quaternion(), Quaternion()};

Vec3f stringScale[2] = {Vec3f(1,2.0f,1),Vec3f(1,2.0f,1)};
Vec3f resetButtonPos, scoreMarkerPos,scoreMarkerOff = Vec3f(101,6,-7);
Vec3f counterPos =  Vec3f(80,0,-100);

int playerHit = 0, playerScore = 0;

bool projOnTarget = false;


void cleanup()
{
	delete mgr;
	delete tracker;
	delete button;
	delete analog;

	beachTrans = NULL;
	mgr = NULL;
	root = NULL; 
	landTrans = NULL;
	standTrans = NULL;
	targetTrans = NULL;
	slingTrans = NULL;
	projectileTrans = NULL;
	pocketTrans = NULL;
	rollTrans = NULL;
	markerTrans = NULL;
	helpTrans = NULL;
	stringTrans[0] =NULL;
	stringTrans[1] =NULL;
	ttrans= NULL;

	helperTrans[0] = NULL;
	helperTrans[1] = NULL;
	helperTrans[2] = NULL;
	helperTrans[3] = NULL;

}

void print_tracker();

#if HELPERS_ON == 1
NodeRecPtr makeHelper(){
	NodeRecPtr boxChild = makeBox(5,5,5,1,1,1);
	SimpleMaterialRecPtr boxMat = SimpleMaterial::create();

	boxMat->setDiffuse(Color3f(1,0.2f,0.1f));
	boxMat->setAmbient(Color3f(0.8f, 0.2f, 0.2f));
	boxMat->setTransparency(0.25);
	//boxMat->setLit(false);

	GeometryRecPtr boxGeo = dynamic_cast<Geometry*>(boxChild->getCore());
	boxGeo->setMaterial(boxMat);
	
	ComponentTransformRecPtr bt = ComponentTransform::create();
	bt->setTranslation(Vec3f(0.f));
	bt->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(0)));

	NodeRecPtr ht = Node::create();
	ht->setCore(bt);
	ht->addChild(boxChild);
	return ht;
}
#endif

NodeRecPtr makeString(SimpleTexturedMaterialRecPtr tex){
	NodeRecPtr boxChild = makeCylinder(20,0.5,6,true,true,true);
	
	GeometryRecPtr boxGeo = dynamic_cast<Geometry*>(boxChild->getCore());
	boxGeo->setMaterial(tex);
	
	ComponentTransformRecPtr bt = ComponentTransform::create();
	bt->setTranslation(Vec3f(0,10,0));

	NodeRecPtr ht = Node::create();
	ht->setCore(bt);
	ht->addChild(boxChild);

	NodeRecPtr wrap = Node::create();
	ComponentTransformRecPtr wt = ComponentTransform::create();
	wrap->setCore(wt);
	wrap->addChild(ht);
	return wrap;
}

NodeRecPtr loadModel(std::string filename, 
	Vec3f trans = Vec3f(0,0,0), 
	float scale = 1.0f, 
	Quaternion rot = Quaternion(Vec3f(1,0,0),osgDegree2Rad(0))) {

		/*

	//model taken from http://storage3d.com/
	NodeRecPtr palmTree = SceneFileHandler::the()->read("models/landscape.obj");

	ComponentTransformRecPtr palmCT = ComponentTransform::create();
	palmCT->setTranslation(Vec3f(0,0,0));
	palmCT->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(0)));
	//palmCT->setScale(Vec3f(10.f,10.f,10.f));

	NodeRecPtr palmTrans = makeNodeFor(palmCT);

	palmTrans->addChild(palmTree);
	
	*/
	
 	NodeRecPtr model = SceneFileHandler::the()->read(filename.c_str());

	ComponentTransformRecPtr modelCT = ComponentTransform::create();
	modelCT->setTranslation(trans);
	modelCT->setRotation(rot);
	modelCT->setScale( Vec3f(scale, scale, scale));

	NodeRecPtr modelTrans = makeNodeFor(modelCT);
	modelTrans->addChild(model);

	return modelTrans;
 }


NodeTransitPtr createScenegraph() {
	NodeRecPtr root = Node::create();
	root->setCore(Group::create());

#if HELPERS_ON == 1
	helpTrans = makeHelper();
	root->addChild(helpTrans);

	helperTrans[0] = makeHelper();
	root->addChild(helperTrans[0]);
	helperTrans[1] = makeHelper();
	root->addChild(helperTrans[1]);
	helperTrans[2] = makeHelper();
	root->addChild(helperTrans[2]);
	helperTrans[3] = makeHelper();
	root->addChild(helperTrans[3]);

	ttrans = makeHelper();	
	root->addChild(ttrans);
#endif


	ImageRecPtr image = Image::create();
	image->read("models/strings_tex.jpg");
	//now we create the texture that will hold the image
	SimpleTexturedMaterialRecPtr tex = SimpleTexturedMaterial::create();
	tex->setImage(image);
	tex->setDiffuse(Color3f(0.8,0.8f,0.8f));

	stringTrans[0] = makeString(tex);
	stringTrans[1] = makeString(tex);

	root->addChild(stringTrans[0]);
	root->addChild(stringTrans[1]);

	GeometryRecPtr sunGeo = makeSphereGeo(2, 3);
	NodeRecPtr sunChild = Node::create();
	sunChild->setCore(sunGeo);
	root->addChild(sunChild);
	//decouple the nodes to be shifted in hierarchy from the scene
	root->subChild(sunChild);

	TransformRecPtr sunTransCore = Transform::create();
	Matrix sunMatrix;
	// Setting up the matrix
	sunMatrix.setIdentity();
	sunMatrix.setTranslate(0,20000,0);
	sunTransCore->setMatrix(sunMatrix); // Adding the Matrix to the core

	// Setting up the node
	NodeRecPtr sunTrans = makeNodeFor(sunTransCore);
	sunTrans->addChild(sunChild);
	root->addChild(sunTrans);
	root->subChild(sunTrans);

	SimpleMaterialRecPtr sunMat = SimpleMaterial::create();
	sunMat->setDiffuse(Color3f(1,0.8f,0));
	sunMat->setAmbient(Color3f(0.8f, 0.2f, 0.2f));
	MaterialGroupRecPtr sunMgCore = MaterialGroup::create();
	sunMgCore->setMaterial(sunMat);
	NodeRecPtr sunMg = Node::create();
	sunMg->setCore(sunMgCore);
	sunMg->addChild(sunTrans);
	root->addChild(sunMg);



	landTrans = loadModel("models/landscape.obj");
	root->addChild(landTrans);

	slingTrans = loadModel("models/slingshot.obj",slingPos, 1.0f);
	root->addChild(slingTrans);
	
	projectileTrans = loadModel("models/projectile.obj");
	root->addChild(projectileTrans);
	
	pocketTrans = loadModel("models/pocket.obj");
	root->addChild(pocketTrans);

	standTrans = loadModel("models/stand.obj", targetPos - Vec3f(0, 159,0));
	root->addChild(standTrans);

	targetTrans = loadModel("models/target.obj", targetPos, 1.0f, Quaternion(Vec3f(1,0,0),osgDegree2Rad(0)));
	//standTrans->addChild(targetTrans);	
	root->addChild(targetTrans);

	NodeRecPtr skybox = loadModel("models/skybox.obj", Vec3f(12,0,0), 10.0f);
	root->addChild(skybox);
	NodeRecPtr counter = loadModel("models/counter.obj",counterPos);
	root->addChild(counter);
	resetButtonPos = counterPos + Vec3f(9, 120, -15);

	rollTrans = loadModel("models/counter_roll.obj", counterPos + Vec3f(45,85,-40));
	root->addChild(rollTrans);
	
	scoreMarkerPos = counterPos + scoreMarkerOff + Vec3f(0,playerScore*0.1,0);
	markerTrans = loadModel("models/score_marker.obj", scoreMarkerPos);
	root->addChild(markerTrans);

	
	#if HELPERS_ON == 1
		ComponentTransformRecPtr ht = dynamic_cast<ComponentTransform*>(helperTrans[3]->getCore());
		ht->setTranslation(resetButtonPos);
	#endif

	PointLightRecPtr sunLight = PointLight::create();
	//sunLight->setAttenuation(1,0,2);
	//color information
	sunLight->setDiffuse(Color4f(1,1,1,1));
	sunLight->setAmbient(Color4f(0.5f,0.5f,0.5f,1));
	sunLight->setSpecular(Color4f(1,1,1,1));
	sunLight->setBeacon(sunChild); //attach to the sun node use this node as position beacon
	root->setCore(sunLight);

	DirectionalLightRecPtr dirLight = DirectionalLight::create();
	dirLight->setDirection(-1,-1,0);

	//color information
	dirLight->setDiffuse(Color4f(1,1,1,1));
	dirLight->setAmbient(Color4f(0.2f,0.2f,0.2f,1));
	dirLight->setSpecular(Color4f(1,1,1,1));

	//wrap the root, cause only nodes below the lights will be lit
	NodeRecPtr ueberroot = makeNodeFor(dirLight);
	ueberroot->addChild(root);

	root = ueberroot;
	return NodeTransitPtr(root);
}

NodeTransitPtr buildScene() {
	return createScenegraph();
}

#define PULL_LOOSE 0
#define PULL_GOING 1
int pullState = PULL_LOOSE;

#define PROJ_DEAD 0
#define PROJ_FLY 1
#define PROJ_PULL 2
int projState = PROJ_DEAD;


void reset(){
	playerHit = 0;
	playerScore=0;
}
void startProjectile(Vec3f pos, Vec3f dir){
	projPos = pos;
	projSpeed = dir*20.f;
	projState = PROJ_FLY;
	
	std::cout << "start projectile" << std::endl;
	std::cout << "projPos: "<< projPos << " 0: "<< projPos[0] << " 1: " << projPos[1] << " 2: " << projPos[2] << std::endl;
	std::cout << "projPos: "<< projPos << " projSpeed: "<< projSpeed << std::endl;
}

void stopProjectile(Vec3f pos){
	projPos = pos;
	projSpeed = Vec3f(0,0,0);
	projState = PROJ_DEAD;
	std::cout << "stop projectile" << std::endl;
}

void eventStartPull(){
	if(pullState == PULL_GOING){
		return;
	}
	
	Vec3f d1 = slingPos - handPos;
	Vec3f d2 = slingPos - headPos;
	float dot =  d1.dot(d2);
	float len = d1.length();
	std::cout << "dot: " << dot<< " length: " << len << std::endl;
	if( len <= 30.0f && len >= 5.f && dot > 0){
		pullState = PULL_GOING;
		projState = PROJ_PULL;
		projOnTarget = false;
		stringFollowSpeed = Vec3f(0,0,0);
		playerHit=0;
		std::cout << "start pull" << std::endl;
	}else{
		
		Vec3f d = resetButtonPos - handPos;
		float len = d.length();
		std::cout << "length: " << len << std::endl;
		if(len <= 20.f){
			std::cout << "reset score"<< std::endl;
			reset();
		}
	}
}

void eventStopPull(){
	if(pullState == PULL_LOOSE){
		return;
	}
	
	Vec3f d1 = slingPoints[SL_AIM] - handPos;	
	startProjectile(handPos, d1);
	pullState = PULL_LOOSE;
	
	std::cout << "stop pull" << std::endl;
}

void updateTarget(){
	targetPos[2] = std::max( std::min(targetPos[2], -100.f), -25*100.f );

	ComponentTransformRecPtr tt = dynamic_cast<ComponentTransform*>(targetTrans->getCore());
	tt->setTranslation(targetPos);
	ComponentTransformRecPtr st = dynamic_cast<ComponentTransform*>(standTrans->getCore());
	st->setTranslation(targetPos- Vec3f(0, 159,0));

	if(projOnTarget == true){
		Vec3f v = Vec3f(projPos[0], projPos[1], targetPos[2]);
		ComponentTransformRecPtr pt = dynamic_cast<ComponentTransform*>(projectileTrans->getCore());
		pt->setTranslation(v);
	}

#if HELPERS_ON == 1	
	ComponentTransformRecPtr ttt = dynamic_cast<ComponentTransform*>(ttrans->getCore());
	ttt->setTranslation(targetPos);
#endif
}

void calcSlingPoints(){
	Vec3f tmpAim = Vec3f(0,0,0);
	slingRot.multVec(slingAimOff, tmpAim);
	
	Vec3f tmpLeft = Vec3f(0,0,0);
	slingRot.multVec(slingAimOff+slingLeftOff, tmpLeft);
	
	Vec3f tmpRight = Vec3f(0,0,0);
	slingRot.multVec(slingAimOff+slingRightOff, tmpRight);

	Vec3f tmpLoose = Vec3f(0,0,0);
	slingRot.multVec(slingAimOff+slingLooseOff, tmpLoose);


	slingPoints[SL_AIM]= slingPos + tmpAim;
	slingPoints[SL_LEFT]= slingPos + tmpLeft;
	slingPoints[SL_RIGHT]= slingPos + tmpRight;
	slingPoints[SL_LOOSE]= slingPos + tmpLoose;
}

void calcStringFollow(float dt){
	Vec3f d = slingPoints[SL_LOOSE] - stringFollow;
	stringFollow += stringFollowSpeed * dt * 240.0f;

	d*= 0.8;
	stringFollowSpeed =(stringFollowSpeed *43.f *dt) + d*dt;
}
/*
void updatePocket_old(){
	auto d = slingPoints[SL_AIM] - stringFollow ;
	d.normalize();
	float angleX = d.dot(Vec3f(1,0,0));
	auto axisX = d.cross(Vec3f(1,0,0));
	auto rotX = Quaternion(axisX, angleX);

	float angleY = d.dot(Vec3f(0,1,0));
	auto axisY = d.cross(Vec3f(0,1,0));
	auto rot = rotX * Quaternion(axisY, angleY);

	rot.multVec(pocketOff[0], pocketPoints[0]);
	rot.multVec(pocketOff[1], pocketPoints[1]);
	pocketPoints[0] += stringFollow;
	pocketPoints[1] += stringFollow;

	ComponentTransformRecPtr pt = dynamic_cast<ComponentTransform*>(pocketTrans->getCore());
	pt->setTranslation(stringFollow);
	pt->setRotation(rot);
}
*/
void updatePocket(){
	auto d = slingPoints[SL_AIM] - stringFollow ;
	d.normalize();

	Vec3f up;
	slingRot.multVec(Vec3f(0,1,0), up);

	float angle = d.dot(up);
	auto axis = d.cross(up);

	auto rot = Quaternion(axis, angle) * slingRot;

	rot.multVec(pocketOff[0], pocketPoints[0]);
	rot.multVec(pocketOff[1], pocketPoints[1]);
	pocketPoints[0] += stringFollow;
	pocketPoints[1] += stringFollow;

	ComponentTransformRecPtr pt = dynamic_cast<ComponentTransform*>(pocketTrans->getCore());
	pt->setTranslation(stringFollow);
	pt->setRotation(rot);
}

#define MAX_SCORE 2000

void updateStand(float dt){
	
	playerScore = std::min(playerScore, MAX_SCORE);
	if(playerScore == MAX_SCORE){
		playerHit = 7;
	}
	ComponentTransformRecPtr st = dynamic_cast<ComponentTransform*>(rollTrans->getCore());
	st->setRotation(Quaternion(Vec3f(1,0,0), osgDegree2Rad(playerHit*45)));


	scoreMarkerPos = counterPos + scoreMarkerOff + Vec3f(0,playerScore*0.1,0);
	
	ComponentTransformRecPtr mt = dynamic_cast<ComponentTransform*>(markerTrans->getCore());
	mt->setTranslation(scoreMarkerPos);

}
void calcStringRotScale(){
	Vec3f d0 = pocketPoints[0] - slingPoints[0];
	Vec3f d1 = pocketPoints[1] - slingPoints[1] ;
	
	auto l0 = d0.length(), 
		l1 = d1.length();

	stringScale[0][1] = l0/20.0f;
	stringScale[1][1] = l1/20.0f;
	
	float thick0 = 0.5 + 5* 1/l0;
	stringScale[0] =Vec3f(thick0, l0/20.0f, thick0);
	float thick1 = 0.5 + 5* 1/l1;
	stringScale[1] =Vec3f(thick1, l1/20.0f, thick1);

	d0.normalize();
	d1.normalize();

	float ang0 =  acosf(Vec3f(0,1,0).dot(d0));
	Vec3f ax0 = Vec3f(0,1,0).cross(d0);

	float ang1 =  acosf(Vec3f(0,1,0).dot(d1));
	Vec3f ax1 = Vec3f(0,1,0).cross(d1);

	stringRot[0] = Quaternion(ax0, ang0);
	stringRot[1] = Quaternion(ax1, ang1);
}

void updateSlingshot(float dt){
	calcSlingPoints();

#if HELPERS_ON == 1
	for(int i = 0; i < 3; i++){
		ComponentTransformRecPtr hlpt = dynamic_cast<ComponentTransform*>(helperTrans[i]->getCore());
		hlpt->setTranslation(slingPoints[i]);		
	}

	ComponentTransformRecPtr ht = dynamic_cast<ComponentTransform*>(helpTrans->getCore());
	ht->setTranslation(handPos);
	ht->setRotation(handRot);

#endif

	ComponentTransformRecPtr st = dynamic_cast<ComponentTransform*>(slingTrans->getCore());
	st->setTranslation(slingPos);
	st->setRotation(slingRot);

	if(pullState == PULL_GOING){
		stringFollow = handPos;
		stringFollowSpeed = Vec3f(0,0,0);	
	}else if(pullState == PULL_LOOSE){
		calcStringFollow(dt);
	}
	updatePocket();
	calcStringRotScale();

	for(int i = 0; i < 2; i++){
		ComponentTransformRecPtr strt = dynamic_cast<ComponentTransform*>(stringTrans[i]->getCore());
		strt->setScale(stringScale[i]);
		strt->setRotation(stringRot[i]);
		strt->setTranslation(slingPoints[i]);		
	}

}

#define VX 0
#define VY 1
#define VZ 2

void updateProjectile(float dt){
	if(projState == PROJ_DEAD){
		return;
	}	
	
	if(projState == PROJ_PULL){
		projPos = handPos;
	}
	
	if(projState == PROJ_FLY){	
		float fact = 0.4f;
		Vec3f gravity = Vec3f(0.0f, -981.f, 0.0f);
		projSpeed += gravity * dt * fact;
		//new position
		Vec3f nPos = projPos + (projSpeed * dt * fact) ;
		

		//std::cout << "projPos: "<< projPos << " projSpeed: "<< projSpeed << " nPos: " << nPos << std::endl;
		//check if target is hit:
		//the flight direction	
	
		if (targetPos[2] < projPos[2] && targetPos[VZ] > nPos[2]){
			Vec3f d = nPos - projPos;
			float t = (float)((float)targetPos[2] - (float)projPos[2]) / (float)d[2];		
			//calc the intersection point:
			Vec3f inter = projPos + (d*t);
			std::cout << "d: " << d	<< " t: " << t <<std::endl;
			Vec3f d2	= targetPos - inter;
			float targetSize = 50.0f;
			float dist = d2.length();
			std::cout <<"targetPos: "<< targetPos << " projPos: "<< projPos << " d2:" << d2 << std::endl;
			std::cout << "targetSize:" << targetSize << " dist: "<< dist << " hit: "<< (dist <= targetSize) << std::endl;
			if(dist <= targetSize){
				int scores[] = {100,75,50,20,10};
				std::string colors[] = {"yellow", "red", "blue", "black", "white"};
				int s = (int) (dist/targetSize * 5.f);
				playerScore += scores[s];
				playerHit = s+1;
				std::cout << "hit target: " << scores[s] << " color: " << colors[s] << std::endl;
				stopProjectile(inter);
				projOnTarget = true;
			}else{				
				playerHit = 6;
				std::cout << "missed target at: " << projPos << " || "<< nPos << " inters:" << inter << std::endl;
			}
		} else {
			projPos = nPos;
		}
		
		if (nPos[VY] <= 0.0f){//check if floor is hit 
			Vec3f d = nPos - projPos;
			float t = (0.0f - (float) projPos[VY]) / d[VY];		
			//calc the intersection point:
			Vec3f inter = projPos + (d*t);
			std::cout << "hit floor: "<< nPos << std::endl;
			stopProjectile(inter);
			playerHit = 6;
		} else {
			projPos = nPos;
		}
	}
	
	
	//update position finally	
	ComponentTransformRecPtr pt = dynamic_cast<ComponentTransform*>(projectileTrans->getCore());
	pt->setTranslation(projPos);
}

template<typename T>
T scale_tracker2cm(const T& value)
{
	static const float scale = OSGCSM::convert_length(cfg.getUnits(), 1.f, OSGCSM::CAVEConfig::CAVEUnitCentimeters);
	return value * scale;
}
void VRPN_CALLBACK callback_head_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	headRot = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	headPos = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}
void VRPN_CALLBACK callback_schleuder_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	slingRot = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	slingPos = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}
void VRPN_CALLBACK callback_hand_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	handRot = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	handPos = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}
void VRPN_CALLBACK callback_analog(void* userData, const vrpn_ANALOGCB analog)
{
	if (analog.num_channel >= 2){
		analog_values = Vec3f(analog.channel[0], 0, -analog.channel[1]);
		targetPos += Vec3f(0,0, analog_values[2]*100);
		std::cout << "analog values: " << analog_values << std::endl;
	}
}
void VRPN_CALLBACK callback_button(void* userData, const vrpn_BUTTONCB button)
{
	if (button.button == 0 && button.state == 1){
		//TODO: check pos and start pulling if true
		eventStartPull();
	}
	if (button.button == 0 && button.state == 0){
		//TODO: check if pulling and release if true
		eventStopPull();
	}
}
void InitTracker(OSGCSM::CAVEConfig &cfg)
{
	try
	{
		const char* const vrpn_name = "DTrack@localhost";
		tracker = new vrpn_Tracker_Remote(vrpn_name);
		tracker->shutup = true;
		tracker->register_change_handler(NULL, callback_head_tracker, cfg.getSensorIDHead());
		tracker->register_change_handler(NULL, callback_hand_tracker, cfg.getSensorIDController());
		tracker->register_change_handler(NULL, callback_schleuder_tracker, 3);
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
	std::cout << "Head position: " << headPos << " orientation: " << headRot << '\n';
	std::cout << "Schleuder position: " << slingPos << " orientation: " << slingRot << '\n';
	std::cout << "Hand position: " << handPos << " orientation: " << handRot << '\n';
	std::cout << "Analog: " << analog_values << '\n';
}

void keyboard(unsigned char k, int x, int y)
{
	Real32 ed;
	float fac = 5.0f;
	switch(k)
	{
		case 'p':
			eventStartPull();
		break;
		case 'o':
			eventStopPull();
		break;
		case 'u':
			targetPos += Vec3f(0,0, -1.0f)*fac;
		break;
		case 'j':
			targetPos += Vec3f(0,0, 1.0f)*fac;
		break;
		case 't':
			slingPos += Vec3f(0,0, -1.0f)*fac;
		break;
		case 'g':
			slingPos += Vec3f(0,0, 1.0f)*fac;
		break;
		case 'c':
			handPos += Vec3f(-1,0, 0)*fac;
		break;
		case 'v':			
			handPos += Vec3f(1,0, 0)*fac;
		break;
		case 'q':
			handPos += Vec3f(0,0,-1)*fac;
		break;
		case 'y':			
			handPos += Vec3f(0,0,1)*fac;
		break;
		case '$':			
			handPos += Vec3f(0,-1,0)*fac;
		break;
		case 'r':
			headRot = Quaternion(Vec3f(0,1,0),3.141f);
			headPos = Vec3f(0.f, 170.f, 200.f);
			reset();
		break;
		case 'w':
			headPos += Vec3f(0,0, -1.0f)*fac;
			break;
		case 'a':
			headPos += Vec3f(-1.0f,0,0)*fac;
			break;
		case 's':
			headPos += Vec3f(0,0, 1.0f)*fac;
			break;
		case 'd':
			headPos += Vec3f(1.0f,0,0 )*fac;
			break;
		case 27: 
			//root->clearChildren();
			cleanup();
			exit(EXIT_SUCCESS);
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
		case 'h':
			cfg.setFollowHead(!cfg.getFollowHead());
			std::cout << "following head: " << std::boolalpha << cfg.getFollowHead() << '\n';
			break;
		case 'i':
			print_tracker();
			break;
		case 'f':
			playerScore += 200;
			std::cout << "score:" << playerScore<< std::endl;
			break;
		default:
			std::cout << "Key '" << k << "' ignored\n";
	}
}

//****** Update LOOP *******//
void updateLoop(float dt){
	updateSlingshot(dt);	
	updateProjectile(dt);	
	updateTarget();
	updateStand(dt);

}

auto t_now= std::chrono::high_resolution_clock::now();
auto t_last = std::chrono::high_resolution_clock::now();
int count = 0;
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

	glutMouseFunc([](int button, int state, int x, int y) {
		
		float angY = 0.5f * ((float) (x-150));
		float angX = 0.5f * ((float) (y-150));
		if(state){
			slingRot = Quaternion(Vec3f(0,1.f,0), osgDegree2Rad(angY));

			slingRot *= Quaternion(Vec3f(1.f,0, 0), osgDegree2Rad(angX));
			
			//rootTrans->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(ang)));
			std::cout << "Mouse: " << x << ", "<< y<< " angleY: " <<  angY << " angleX: "<< angX << '\n';
		}
		glutPostRedisplay();
	});
	
	glutIdleFunc([]()
	{

		check_tracker();
		const auto speed = 1.f;
		mgr->setUserTransform(headPos, headRot);
		mgr->setTranslation(mgr->getTranslation());
		//mgr->setTranslation(mgr->getTranslation() + speed * analog_values);
		t_last = t_now;
		t_now= std::chrono::high_resolution_clock::now();
		float dt =  (float) std::chrono::duration<double>(t_now-t_last).count();
		count++;
		if(count % 9000 == 0){
			std::cout<< dt<<std::endl;
		}
		//update
		updateLoop(dt);
		
		
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

		if (!scene) 
			scene = buildScene();
		commitChanges();

		mgr = new OSGCSM::CAVESceneManager(&cfg);
		mgr->setWindow(mwin );
		mgr->setRoot(scene);
		mgr->setHeadlight(false);
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
