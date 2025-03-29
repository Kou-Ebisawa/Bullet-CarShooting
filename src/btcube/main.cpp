/*!
  @file main.cpp

  @brief メインファイル(GLFWウィンドウ生成など)

  @author Makoto Fujisawa
  @date   2021-11
*/

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "glfw3dll.lib")

#ifdef _DEBUG
#pragma comment (lib, "LinearMath_vs2010_x64_debug.lib")
#pragma comment (lib, "BulletCollision_vs2010_x64_debug.lib")
#pragma comment (lib, "BulletDynamics_vs2010_x64_debug.lib")
#pragma comment (lib, "BulletSoftBody_vs2010_x64_debug.lib")
#else
#pragma comment (lib, "LinearMath_vs2010_x64_release.lib")
#pragma comment (lib, "BulletCollision_vs2010_x64_release.lib")
#pragma comment (lib, "BulletDynamics_vs2010_x64_release.lib")
#pragma comment (lib, "BulletSoftBody_vs2010_x64_release.lib")
#endif


#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

#define GL_SILENCE_DEPRECATION	// mac環境でgluを使っている場合の非推奨warningの抑制

//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "utils.h"

// ImGUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include "time.h"

//文字列描画
#pragma comment (lib, "freetype.lib")
#pragma comment (lib, "ftgl.lib")
#include <FTGL/ftgl.h>
#define FONT_FILE "Inconsolata.ttf"
static FTPixmapFont* g_font;
unsigned long g_fontsize = 18;  // フォントサイズ

#include <cstdlib>

#define SPAWN 3000//敵スポーン間隔の設定
#define FIRE 100//操作オブジェクトの発射可能間隔
#define ENEMYFIRE 500//敵オブジェクトの発射間隔
#define CLEAR 5 //クリアの基準

using namespace std;


//-----------------------------------------------------------------------------
// 定数・グローバル変数
//-----------------------------------------------------------------------------
const glm::vec4 LIGHT0_POS(0.5f, 4.0f, 1.5f, 0.0f);
const glm::vec4 LIGHT1_POS(-1.0f, -10.0f, -1.0f, 0.0f);
const glm::vec4 LIGHT_AMBI(0.3f, 0.3f, 0.3f, 1.0f);
const glm::vec4 LIGHT_DIFF(1.0f, 1.0f, 1.0f, 1.0f);
const glm::vec4 LIGHT_SPEC(1.0f, 1.0f, 1.0f, 1.0f);
const GLfloat FOV = 45.0f;

// グローバル変数
int g_winw = 1024;							//!< 描画ウィンドウの幅
int g_winh = 1024;							//!< 描画ウィンドウの高さ
rxTrackball g_view;							//!< 視点移動用トラックボール
float g_bgcolor[3] = { 1, 1, 1 };			//!< 背景色
bool g_animation_on = false;				//!< アニメーションON/OFF
int g_currentstep = 0;						//!< 現在のステップ数
const int MAX_CONFLICT_POINTS = 1024; //衝突点数の配列のサイズ
btVector3 g_confliction_points[MAX_CONFLICT_POINTS]; //衝突点数を格納する配列
btHingeConstraint* g_constraint_c = 0;//車体と弾の発射部分の条件をつけるConstraintを保存
//タイヤと車体との間の条件を付けるConstraintを保存
btHingeConstraint* g_constraint1=0; 
btHinge2Constraint* g_constraint2 = 0;
btHingeConstraint* g_constraint3 = 0;
btHinge2Constraint* g_constraint4 = 0;
//タイヤにモーターを設定
btRotationalLimitMotor2* g_motor1;
btRotationalLimitMotor2* g_motor2;
//タイヤに横方向のモーターを設定
btRotationalLimitMotor2* g_motor12;
btRotationalLimitMotor2* g_motor22;

double g_firingangle = 0.0;//発射位置の修正
double g_targetsteerangle = 0.0;//タイヤ横方向の動き
double g_targetvelocity = 0.0;//タイヤ前方向(モーター)の動き
// シャドウマッピング
ShadowMap g_shadowmap;
int g_shadowmap_res = 1024;

// 物理シミュレーション関連定数/変数
float g_dt = 0.002;	//!< 時間ステップ幅

// Bullet
btSoftRigidDynamicsWorld* g_dynamicsworld;	//!< Bulletワールド
btAlignedObjectArray<btCollisionShape*>	g_collisionshapes;		//!< 剛体オブジェクトの形状を格納する動的配列
btSoftBodyWorldInfo g_softBodyWorldInfo;

// マウスピック
btVector3 g_pickpos;
btRigidBody *g_pickbody = 0;
btPoint2PointConstraint *g_pickconstraint = 0;
btSoftBody::Node *g_picknode = 0;
double g_pickdist = 0.0;
//操作オブジェクトに対するグローバル変数
btRigidBody* g_tracerbody;//操作対象
btRigidBody* g_ballpositionbody;//弾の発射部分

//敵のオブジェクトを格納するグローバル変数
btRigidBody* g_enemybody_1;
btRigidBody* g_enemybody_2;
btRigidBody* g_enemybody_3;
double g_enemyspeed = 6;//敵の移動速度
int g_wait_time = 0;//発射可能な時間までの待ち時間

//敵オブジェクト1の管理に利用
bool g_turnflag = false;
int g_step = 0;
int g_tmp = 2;
//敵オブジェクト2
bool g_turnflag2 = false;
int g_step2 = 0;
int g_tmp2 = 2;
//敵オブジェクト3
bool g_turnflag3 = false;
int g_step3 = 0;
int g_tmp3 = 2;

//ダメージの際の文字列表示に利用
int g_str_tmp = 0;
bool g_str_tmp_flag = false;
bool g_str_flag = false;

//ポイントの際の文字列表示に利用
int g_str_tmp2 = 0;
bool g_str_tmp_flag2 = false;
bool g_str_flag2 = false;

int g_spawntime = 0;//敵オブジェクトの生成間隔をカウント

int g_points = 0;//プレイヤーの得点
int g_life = 3;//プレイヤーのライフ
static bool g_clear_flag = true;//クリアフラグ
static bool g_over_flag = false;//オーバーフラグ
static bool g_end_flag = false;//ゲーム終了フラグ
static bool g_back_view = false;//視点を背後に固定

static bool g_start_flag = true;//ゲーム開始をきめるフラグ

//-----------------------------------------------------------------------------
// Bullet用関数
//-----------------------------------------------------------------------------
/*!
* Bullet剛体(btRigidBody)の作成
* @param[in] mass 質量
* @param[in] init_tras 初期位置・姿勢
* @param[in] shape 形状
* @return 作成したbtRigidBody
*/
btRigidBody* CreateRigidBody(double mass, const btTransform& init_trans, btCollisionShape* shape, btDynamicsWorld* world = 0, int index = 0)
{
	//btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	// 質量が0ならば静的な(static)オブジェクトとして設定，
	bool isDynamic = (mass != 0.0);

	btVector3 inertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, inertia);

	btDefaultMotionState* motion_state = new btDefaultMotionState(init_trans);

	btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shape, inertia);

	btRigidBody* body = new btRigidBody(rb_info);

	body->setUserIndex(index);

	if (mass <= 1e-10) {
		// Kinematicオブジェクトとして設定(stepSimulationしても運動の計算を行わないようにする)
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		// 常にスリープ状態にする
		//body->setActivationState(DISABLE_DEACTIVATION);
	}

	
	enum CollisionGroup {//衝突するオブジェクトを決定
		RX_COL_GROUND = 1,
		RX_COL_CAR=2,
		RX_COL_BALL=4,
		RX_COL_ENEMY=8,
		RX_COL_ENEMY_BALL=16,
		RX_COL_SPECIAL=32,
		RX_COL_SPECIAL_BALL=64,
	};
	if (world) {
		if (body->getUserIndex() == 259) {//操作オブジェクトの衝突判定
			world->addRigidBody(body, RX_COL_CAR, RX_COL_GROUND|RX_COL_ENEMY_BALL|RX_COL_SPECIAL);
		}
		else if (body->getUserIndex() == 100) {//操作オブジェクトの弾の衝突判定
			world->addRigidBody(body, RX_COL_BALL, RX_COL_GROUND|RX_COL_ENEMY|RX_COL_ENEMY_BALL|RX_COL_SPECIAL);
		}
		else if (body->getUserIndex() == 10) {//敵オブジェクトの衝突判定
			world->addRigidBody(body, RX_COL_ENEMY, RX_COL_GROUND | RX_COL_BALL);
		}
		else if (body->getUserIndex() == 200) {//敵オブジェクトの弾の衝突判定
			world->addRigidBody(body, RX_COL_ENEMY_BALL, RX_COL_GROUND | RX_COL_BALL | RX_COL_CAR);
		}
		else if (body->getUserIndex() == 15) {//敵特殊オブジェクトの衝突判定
			world->addRigidBody(body, RX_COL_ENEMY, RX_COL_GROUND | RX_COL_BALL);
		}
		else if (body->getUserIndex() == 250) {//敵特殊オブジェクトの衝突判定
			world->addRigidBody(body, RX_COL_ENEMY_BALL, RX_COL_GROUND | RX_COL_BALL | RX_COL_CAR);
		}
		else {//地面の衝突判定
			world->addRigidBody(body, RX_COL_GROUND, RX_COL_GROUND | RX_COL_CAR | RX_COL_BALL|RX_COL_ENEMY|RX_COL_ENEMY_BALL|RX_COL_SPECIAL|RX_COL_SPECIAL_BALL);
		}
	}

	return body;
}

btVector3 Ball_position(btRigidBody* body1, btRigidBody* body2) {//プレイヤーの発射する弾の位置を決定
	btTransform trans;
	body1->getMotionState()->getWorldTransform(trans);
	btVector3 p = trans.getOrigin();
	btTransform trans2;
	body2->getMotionState()->getWorldTransform(trans2);
	btVector3 p_body = trans2.getOrigin();
	btQuaternion q = trans2.getRotation();
	btVector3 ball_direction = p - (p_body + quatRotate(q, btVector3(0, 0.4, 0.2)));
	ball_direction.normalize();
	btVector3 epos = p + ball_direction * 0.4;
	return epos;
}

btVector3 Ball_direction(btRigidBody* body1, btRigidBody* body2) {//プレイヤーの発射する弾の方向を決定
	btTransform trans;
	body1->getMotionState()->getWorldTransform(trans);
	btVector3 p = trans.getOrigin();
	btTransform trans2;
	body2->getMotionState()->getWorldTransform(trans2);
	btQuaternion q = trans2.getRotation();
	btVector3 p_body = trans2.getOrigin();
	btVector3 ball_direction = p - (p_body + quatRotate(q, btVector3(0, 0.4, 0.2)));
	ball_direction.normalize();
	return ball_direction;
}

btVector3 Enemy_Ball_position(btRigidBody* body1) {//敵オブジェクトの発射する弾の位置を決定
	btTransform trans;
	body1->getMotionState()->getWorldTransform(trans);
	btVector3 p = trans.getOrigin();
	btVector3 epos = p + btVector3(0.0, 0.5, 0.0);//敵オブジェクトは向いている方向に射撃するので、中心より0.5上方向から発射
	return epos;
}

btVector3 Enemy_Ball_direction(btRigidBody* body) {//敵オブジェクトの発射する弾の方向を決定
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	double one_c = glm::pi<double>() / 180.0;
	double ang = trans.getRotation().getAngle();
	double yaw, pitch, roll;
	trans.getRotation().getEulerZYX(yaw, pitch, roll);
	//z軸方向かどうかを判別
	if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
		//z軸負方向かどうかを判別
		if (btDegrees(ang) <= 45)
			return btVector3(0, 0, -1);
		else
			return btVector3(0,0,1);
	}
	//x軸負方向かどうかを判別
	else if (btDegrees(pitch) > 45) {
		return btVector3(-1,0,0);
	}
	else {
		return btVector3(1,0,0);//x軸正方向の処理
	}

}

/*!
 * 文字列描画
 * @param[in] str 文字列
 * @param[in] w,h ウィンドウサイズ
 * @param[in] x0,y0 文字列の位置(左上原点のスクリーン座標系,文字列の左下がこの位置になる)
 */
void DrawStrings(vector<string>& strs, int w, int h, int x0, int y0, unsigned long fontsize)
{
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glRasterPos2f(x0, y0);

	// フォントの初期化
	if (!g_font) {
		g_font = new FTPixmapFont(FONT_FILE);
		if (g_font->Error()) {
			cout << "Failed to open font " << FONT_FILE << endl;
			delete g_font;
			g_font = 0;
		}
		else {
			g_font->FaceSize(fontsize);
		}
	}

	// FTGLで文字列を描画
	if (g_font) {
		g_font->FaceSize(fontsize);
		for (int j = 0; j < (int)strs.size(); ++j) {
			glRasterPos2f(x0, y0);
			strs[j].push_back('\0');
			g_font->Render(strs[j].c_str());
			y0 += g_font->LineHeight();
		}
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

btRigidBody* CreateEnemyBody(int idx,btVector3 pos, btQuaternion rot){//二つの変数を受け取り、その位置、方向で敵オブジェクトを生成
	const btScalar CUBE_HALF_EXTENTS = 0.2;	// 立方体の変の長さの半分(中心から辺までの距離)
	const btScalar GROUND_HEIGHT = 0.0;		// 地面の高さ
	btCollisionShape* x_cylinder_shape = new btCylinderShapeX(btVector3(0.1, 0.2, 0.2));
	btTransform trans;	// 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity();// 位置姿勢行列の初期化
	btCollisionShape* box_shape = new btBoxShape(btVector3(2 * CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, 4 * CUBE_HALF_EXTENTS));
	g_collisionshapes.push_back(box_shape); // 最後に破棄(delete)するために形状データを格納しておく

	btCollisionShape* c_cylinder_shape = new btCylinderShape(btVector3(0.3, 0.15, 0.15));
	g_collisionshapes.push_back(c_cylinder_shape);

	btCollisionShape* c_box_shape = new btBoxShape(btVector3(1.5 * CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS / 4 + 0.025, 2.5 * CUBE_HALF_EXTENTS));
	g_collisionshapes.push_back(c_box_shape);

	btCollisionShape* capsule_shape = new btCapsuleShape(btScalar(0.025), btScalar(0.5));
	g_collisionshapes.push_back(capsule_shape); // 最後に破棄(delete)するために形状データを格納しておく

	btCollisionShape* c_zcylinder_shape = new btCylinderShapeZ(btVector3(0.115, -0.4, -0.4));

	btCompoundShape* compound = new btCompoundShape;

	btQuaternion qrot(0, 0, 0, 1);
	//車体の追加
	trans.setOrigin(btVector3(0, GROUND_HEIGHT, 0));
	trans.setRotation(qrot);// 四元数を行列に変換して姿勢行列に掛け合わせる
	compound->addChildShape(trans, box_shape);
	//車体の上の小さな段差
	trans.setOrigin(btVector3(0, GROUND_HEIGHT + CUBE_HALF_EXTENTS, 0.2));
	trans.setRotation(qrot);
	compound->addChildShape(trans, c_box_shape);
	//旗の棒部分
	trans.setOrigin(btVector3(0.3, GROUND_HEIGHT + CUBE_HALF_EXTENTS + 0.2, -0.7));
	trans.setRotation(qrot);
	compound->addChildShape(trans, capsule_shape);
	//砲台部分
	trans.setOrigin(btVector3(0, GROUND_HEIGHT + 0.1 + CUBE_HALF_EXTENTS * 1.5, 0.2));
	trans.setRotation(qrot);
	compound->addChildShape(trans, c_cylinder_shape);
	//弾の射出部分
	trans.setOrigin(btVector3(0, GROUND_HEIGHT + CUBE_HALF_EXTENTS * 1.5 + 0.1, -0.1));
	trans.setRotation(qrot);
	compound->addChildShape(trans, c_zcylinder_shape);
	//タイヤ部分
	trans.setOrigin(btVector3(2 * CUBE_HALF_EXTENTS + 0.1, GROUND_HEIGHT -0.2, 3 * CUBE_HALF_EXTENTS));//位置+,+ 
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる
	compound->addChildShape(trans, x_cylinder_shape);
	trans.setOrigin(btVector3(2 * CUBE_HALF_EXTENTS + 0.1, GROUND_HEIGHT - 0.2, -3 * CUBE_HALF_EXTENTS));//位置+,- ハンドルR
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる
	compound->addChildShape(trans, x_cylinder_shape);
	trans.setOrigin(btVector3(-2 * CUBE_HALF_EXTENTS - 0.1, GROUND_HEIGHT - 0.2, 3 * CUBE_HALF_EXTENTS));//位置-,+ 
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる
	compound->addChildShape(trans, x_cylinder_shape);
	trans.setOrigin(btVector3(-2 * CUBE_HALF_EXTENTS - 0.1, GROUND_HEIGHT - 0.2, -3 * CUBE_HALF_EXTENTS));//位置-,- ハンドルL
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる
	compound->addChildShape(trans, x_cylinder_shape);
	//compoundの計算
	compound->recalculateLocalAabb();
	trans.setIdentity();
	trans.setOrigin(pos);
	trans.setRotation(rot);
	btRigidBody* body = CreateRigidBody(1.0, trans, compound, g_dynamicsworld, idx);
	body->setFriction(0);

	// すり抜け防止用Swept sphereの設定(CCD:Continuous Collision Detection)
	body->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
	body->setCcdSweptSphereRadius(0.05 * CUBE_HALF_EXTENTS);
	return body;
}

void SetEnemydir(btRigidBody* body,int turn_dir) {//敵オブジェクトの回転に関する設定
	//右回転
	body->setLinearVelocity(btVector3(0, 0, 0));//回転中は動きストップ
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	if (turn_dir == 0) {//turn_dirが0なら反時計回り
		//trans.setRotation(trans.getRotation() * (btQuaternion(btRadians(0.5),0,0)).normalize());
		body->setAngularVelocity(btVector3(0, -1.1, 0));
		body->setCenterOfMassTransform(trans);
	}
	else {
		//trans.setRotation(trans.getRotation() * (btQuaternion(btRadians(-0.5),0,0)).normalize());
		body->setAngularVelocity(btVector3(0, 1.1, 0));
		body->setCenterOfMassTransform(trans);
	}
}

void SpawnEnemyObject(void) {//敵オブジェクトの生成条件を設定
	srand((unsigned int)time(NULL));
	double p = rand() % 4;
	if(g_start_flag==false)
		g_spawntime++;
	if (g_enemybody_1==0) {//敵オブジェクト1の生成条件
		if (p == 0) {
			g_enemybody_1 = CreateEnemyBody(10, btVector3(-15.75, 0.5, 15.75), btQuaternion(btRadians(0), 0, 0).normalize());//敵オブジェクト
			g_enemybody_1->setLinearVelocity(btVector3(0, 0, -g_enemyspeed));
		}
		if (p == 1) {
			g_enemybody_1= CreateEnemyBody(10, btVector3(-15.75, 0.5, -15.75), btQuaternion(btRadians(180), 0, 0).normalize());//敵オブジェクト
			g_enemybody_1->setLinearVelocity(btVector3(0, 0, g_enemyspeed));
		}
		if (p == 2) {
			g_enemybody_1 = CreateEnemyBody(10, btVector3(15.75, 0.5, -15.75), btQuaternion(btRadians(180), 0, 0).normalize());//敵オブジェクト
			g_enemybody_1->setLinearVelocity(btVector3(0, 0, g_enemyspeed));
		}
		if (p == 3) {
			g_enemybody_1 = CreateEnemyBody(10, btVector3(15.75, 0.5, 15.75), btQuaternion(btRadians(0), 0, 0).normalize());//敵オブジェクト
			g_enemybody_1->setLinearVelocity(btVector3(0, 0, -g_enemyspeed));
		}
	}
	if (g_enemybody_2==0&&g_spawntime>SPAWN) {//敵オブジェクト2の生成条件
		if (p == 0) {
			g_enemybody_2 = CreateEnemyBody(10, btVector3(-15.75, 0.5, 15.75), btQuaternion(btRadians(0), 0, 0).normalize());//敵オブジェクト
			g_enemybody_2->setLinearVelocity(btVector3(0, 0, -g_enemyspeed));
		}
		if (p == 1) {
			g_enemybody_2 = CreateEnemyBody(10, btVector3(-15.75, 0.5, -15.75), btQuaternion(btRadians(180), 0, 0).normalize());//敵オブジェクト
			g_enemybody_2->setLinearVelocity(btVector3(0, 0, g_enemyspeed));
		}
		if (p == 2) {
			g_enemybody_2 = CreateEnemyBody(10, btVector3(15.75, 0.5, -15.75), btQuaternion(btRadians(180), 0, 0).normalize());//敵オブジェクト
			g_enemybody_2->setLinearVelocity(btVector3(0, 0, g_enemyspeed));
		}
		if (p == 3) {
			g_enemybody_2 = CreateEnemyBody(10, btVector3(15.75, 0.5, 15.75), btQuaternion(btRadians(0), 0, 0).normalize());//敵オブジェクト
			g_enemybody_2->setLinearVelocity(btVector3(0, 0, -g_enemyspeed));
		}
		g_spawntime = 0;
	}
	if (g_enemybody_3==0 && g_spawntime > SPAWN * 1.5 && g_points>=2 ) {//敵オブジェクト3の生成条件、user_idxを15に変更
		if (p == 0) {
			g_enemybody_3 = CreateEnemyBody(15, btVector3(-15.75, 0.5, 15.75), btQuaternion(btRadians(0), 0, 0).normalize());//敵オブジェクト
			g_enemybody_3->setLinearVelocity(btVector3(0, 0, -g_enemyspeed));
		}
		if (p == 1) {
			g_enemybody_3 = CreateEnemyBody(15, btVector3(-15.75, 0.5, -15.75), btQuaternion(btRadians(180), 0, 0).normalize());//敵オブジェクト
			g_enemybody_3->setLinearVelocity(btVector3(0, 0, g_enemyspeed));
		}
		if (p == 2) {
			g_enemybody_3 = CreateEnemyBody(15, btVector3(15.75, 0.5, -15.75), btQuaternion(btRadians(180), 0, 0).normalize());//敵オブジェクト
			g_enemybody_3->setLinearVelocity(btVector3(0, 0, g_enemyspeed));
		}
		if (p == 3) {
			g_enemybody_3 = CreateEnemyBody(15, btVector3(15.75, 0.5, 15.75), btQuaternion(btRadians(0), 0, 0).normalize());//敵オブジェクト
			g_enemybody_3->setLinearVelocity(btVector3(0, 0, -g_enemyspeed));
		}
		g_spawntime = 0;
	}

}

void Enemyturn(btRigidBody* body, int turn_dir) {//敵オブジェクトの回転後に動く方向の設定
	btVector3 vel = body->getLinearVelocity();
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 pos = trans.getOrigin();
	btQuaternion qrot = trans.getRotation();
	btQuaternion q;
	btTransform new_trans;
	double one_c = glm::pi<double>() / 180.0;
	double ang = trans.getRotation().getAngle();
	double yaw, pitch, roll;
	trans.getRotation().getEulerZYX(yaw, pitch, roll);

	//z軸方向かどうかを判別
	if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
		//z軸負方向かどうかを判別
		if (btDegrees(ang) <= 45) {
			body->setLinearVelocity(btVector3(0, 0, -g_enemyspeed));
			q = btQuaternion(btRadians(0), 0, 0).normalize();
		}
		//z軸正方向の処理
		else {
			body->setLinearVelocity(btVector3(0, 0, g_enemyspeed));
			q = btQuaternion(btRadians(180), 0, 0).normalize();
		}
	}
	//x軸負方向かどうかを判別
	else if (btDegrees(pitch) > 45) {
		body->setLinearVelocity(btVector3(-g_enemyspeed, 0, 0));
		q = btQuaternion(btRadians(90), 0, 0).normalize();
	}
	else {
		body->setLinearVelocity(btVector3(g_enemyspeed, 0, 0));
		q = btQuaternion(btRadians(270), 0, 0).normalize();
	}

	double x_pos;
	if (pos[0] < -12.5) {
		x_pos = -15.75;
		if (pos[2] < -12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -15.75));
		}
		else if (pos[2] >= -7.5 && pos[2] < -2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -5));
		}
		else if (pos[2] >= -2.5 && pos[2] < 2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 0));
		}
		else if (pos[2] >= 2.5 && pos[2] < 7.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 5));
		}
		else if (pos[2] >= 12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 15.75));
		}
		else return;
	}
	else if (pos[0] >= -7.5 && pos[0] < -2.5) {
		x_pos = -5;
		if (pos[2] < -12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -15.75));
		}
		else if (pos[2] >= -7.5 && pos[2] < -2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -5));
		}
		else if (pos[2] >= -2.5 && pos[2] < 2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 0));
		}
		else if (pos[2] >= 2.5 && pos[2] < 7.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 5));
		}
		else if (pos[2] >= 12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 15.75));
		}
		else return;
	}
	else if (pos[0] >= -2.5 && pos[0] < 2.5) {
		x_pos = 0;
		if (pos[2] < -12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -15.75));
		}
		else if (pos[2] >= -7.5 && pos[2] < -2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -5));
		}
		else if (pos[2] >= -2.5 && pos[2] < 2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 0));
		}
		else if (pos[2] >= 2.5 && pos[2] < 7.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 5));
		}
		else if (pos[2] >= 12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 15.75));
		}
		else return;
	}
	else if (pos[0] >= 2.5 && pos[0] < 7.5) {
		x_pos = 5;
		if (pos[2] < -12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -15.75));
		}
		else if (pos[2] >= -7.5 && pos[2] < -2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -5));
		}
		else if (pos[2] >= -2.5 && pos[2] < 2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 0));
		}
		else if (pos[2] >= 2.5 && pos[2] < 7.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 5));
		}
		else if (pos[2] >= 12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 15.75));
		}
		else return;
	}
	else if (pos[0] >= 12.5) {
		x_pos = 15.75;
		if (pos[2] < -12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -15.75));
		}
		else if (pos[2] >= -7.5 && pos[2] < -2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], -5));
		}
		else if (pos[2] >= -2.5 && pos[2] < 2.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 0));
		}
		else if (pos[2] >= 2.5 && pos[2] < 7.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 5));
		}
		else if (pos[2] >= 12.5) {
			new_trans = btTransform(q, btVector3(x_pos, pos[1], 15.75));
		}
		else return;
	}
	body->setCenterOfMassTransform(new_trans);
}

int SetDirection(btRigidBody* body) {//敵オブジェクトの回転の向きを決定、return 0なら時計に回転、1なら反時計に回転、2なら変更なし
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 pos = trans.getOrigin();
	btQuaternion qrot = trans.getRotation();
	btVector3 vel = body->getLinearVelocity();
	double one_c = glm::pi<double>() / 180.0;
	double ang = trans.getRotation().getAngle();
	double yaw, pitch, roll;
	trans.getRotation().getEulerZYX(yaw, pitch, roll);
	srand((unsigned int)time(NULL));
	int r1;//時計回りまたは直進
	if (rand() % 2 == 0)
		r1 = 1;
	else
		r1 = 2;
	int r2;//反時計回りまたは直進
	if (rand() % 2 == 0)
		r2 = 0;
	else
		r2 = 2;

	body->setLinearVelocity(btVector3(vel));
	if ((pos[0] >= 2.5 && pos[0] < 7.5) || (pos[0] >= -7.5 && pos[0] < -2.5)||(pos[0]>=-2.5&&pos[0]<=2.5)) {
		if ((pos[2] >= -7.5 && pos[2] < -2.5) || (pos[2] >= 2.5 && pos[2] < 7.5) || (pos[2] >= -2.5 && pos[2] <= 2.5)) {//向きに関係なく、方向自由、回転自由
			return rand() % 3;
		}
		else if (pos[2] < -12.5) {//一番上のマス3つ
			//z軸方向かどうかを判別
			if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
				//z軸負方向かどうかを判別
				if (btDegrees(ang) <= 45)
					return rand() % 2;
				else
					return 2;
			}
			//x軸負方向かどうかを判別
			else if (btDegrees(pitch) > 45) {
				return r2;
			}
			else {
				return r1;//x軸正方向の処理
			}
		}
		else if (pos[2] >= 12.5) {
			//z軸方向かどうかを判別
			if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
				//z軸負方向かどうかを判別
				if (btDegrees(ang) <= 45)
					return 2;
				//z軸正方向の処理
				else
					return rand()%2;
			}
			//x軸負方向かどうかを判別
			else if (btDegrees(pitch) > 45) {
				return r1;//直進または時計回り
			}
			else {
				return r2;//x軸正方向の処理
			}
		}
		else 
			return 2;
	}
	else if (pos[0] < -12.5) {
		if ((pos[2] >= -7.5 && pos[2] < -2.5) || (pos[2] >= 2.5 && pos[2] < 7.5) || (pos[2] >= -2.5 && pos[2] <= 2.5)) {
			//z軸方向かどうかを判別
			if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
				//z軸負方向かどうかを判別
				if (btDegrees(ang) <= 45)
					return r1;
				//z軸正方向の処理
				else
					return r2;
			}
			//x軸負方向かどうかを判別
			else if (btDegrees(pitch) > 45) {
				return rand() % 2;//直進または時計回り
			}
			else {
				return 2;//x軸正方向の処理
			}
		}
		else if (pos[2] < -12.5) {
			//z軸方向かどうかを判別
			if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
				//z軸負方向かどうかを判別
				if (btDegrees(ang) <= 45)
					return 1;
				//z軸正方向の処理
				else
					return 2;
			}
			//x軸負方向かどうかを判別
			else if (btDegrees(pitch) > 45) {
				return 0;//直進または時計回り
			}
			else {
				return 2;//x軸正方向の処理
			}
		}
		else if (pos[2] >= 12.5) {
			//z軸方向かどうかを判別
			if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
				//z軸負方向かどうかを判別
				if (btDegrees(ang) <= 45)
					return 2;
				//z軸正方向の処理
				else
					return 0;
			}
			//x軸負方向かどうかを判別
			else if (btDegrees(pitch) > 45) {
				return 1;//直進または時計回り
			}
			else {
				return 2;//x軸正方向の処理
			}
		}
		else 
			return 2;
	}
	else if (pos[0] >= 12.5) {//方向右固定
		if ((pos[2] >= -7.5 && pos[2] < -2.5) || (pos[2] >= 2.5 && pos[2] < 7.5) || (pos[2] >= -2.5 && pos[2] <= 2.5)) {
			//z軸方向かどうかを判別
			if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
				//z軸負方向かどうかを判別
				if (btDegrees(ang) <= 45)
					return r2;
				//z軸正方向の処理
				else
					return r1;
			}
			//x軸負方向かどうかを判別
			else if (btDegrees(pitch) > 45) {
				return 2;//直進または時計回り
			}
			else {
				return rand()%2;//x軸正方向の処理
			}
		}
		else if (pos[2] < -12.5) {
			//z軸方向かどうかを判別
			if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
				//z軸負方向かどうかを判別
				if (btDegrees(ang) <= 45)
					return 0;
				//z軸正方向の処理
				else
					return 2;
			}
			//x軸負方向かどうかを判別
			else if (btDegrees(pitch) > 45) {
				return 2;//直進または時計回り
			}
			else {
				return 1;//x軸正方向の処理
			}
		}
		else if (pos[2] >= 12.5) {
			//z軸方向かどうかを判別
			if (-45 < btDegrees(pitch) && btDegrees(pitch) <= 45) {
				//z軸負方向かどうかを判別
				if (btDegrees(ang) <= 45)
					return 2;
				//z軸正方向の処理
				else
					return 1;
			}
			//x軸負方向かどうかを判別
			else if (btDegrees(pitch) > 45) {
				return 2;//直進または時計回り
			}
			else {
				return 0;//x軸正方向の処理
			}
		}
		else 
			return 2;
	}
	else //どれにも当てはまらなかったら直進
		return 2;
}

int enemypos(btRigidBody* body) {//敵オブジェクトの位置が回転処理必要かを判定
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 pos = trans.getOrigin();
	if ((pos[0] <= -15.65 && pos[0] >= -15.85) || (pos[0] <= -4.9 && pos[0] >= -5.1) || (pos[0] <= 5.1 && pos[0] >= 4.9) || (pos[0] >=15.65&& pos[0] <= 15.85||(pos[0]>=-0.1&&pos[0]<=0.1)))
		if ((pos[2] <= -15.65 && pos[2] >= -15.85) || (pos[2] <= -4.9 && pos[2] >= -5.1) || (pos[2] <= 5.1 && pos[2] >= 4.9) || (pos[2] >= 15.65 && pos[2] <= 15.85)||(pos[2]>=-0.1&&pos[2]<=0.1))
			return 1;
	return 0;
}

/*!
* 剛体オブジェクトの追加
*/
void SetRigidBodies(void)
{
	btTransform trans;	// 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity();// 位置姿勢行列の初期化

	const btScalar CUBE_HALF_EXTENTS = 0.2;	// 立方体の変の長さの半分(中心から辺までの距離)
	const btScalar GROUND_HEIGHT = 0.0;		// 地面の高さ

	// ----- 地面(質量0のx-z平面上で平べったい直方体で表現)の追加 -----
	btCollisionShape *ground_shape = new btBoxShape(btVector3(20, CUBE_HALF_EXTENTS, 20));	// 形状
	g_collisionshapes.push_back(ground_shape); // 最後に破棄(delete)するために形状データを格納しておく

	ground_shape->setUserIndex(99); // 99とした場合のみ描画時にテクスチャ付き平面として描画．床を傾けたい等の場合は99にしないこと．
	trans.setOrigin(btVector3(0, GROUND_HEIGHT-CUBE_HALF_EXTENTS, 0));	// 上の面がy=0になるように設定
	
	// 剛体オブジェクト(Static)生成
	btRigidBody* body0 = CreateRigidBody(0.0, trans, ground_shape, g_dynamicsworld, 99);

	//床のx,zはともに40
	// ----- ここまで (地面の追加) -----

	// ----- 立方体オブジェクト追加 -----
	btCollisionShape* box_shape = new btBoxShape(btVector3(2*CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, 4*CUBE_HALF_EXTENTS));
	g_collisionshapes.push_back(box_shape); // 最後に破棄(delete)するために形状データを格納しておく

	btCollisionShape* c_cylinder_shape = new btCylinderShape(btVector3(0.3, 0.15, 0.15));
	g_collisionshapes.push_back(c_cylinder_shape);

	btCollisionShape* c_box_shape = new btBoxShape(btVector3(1.5 * CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS / 4+0.025, 2.5 * CUBE_HALF_EXTENTS));
	g_collisionshapes.push_back(c_box_shape);

	btCollisionShape* capsule_shape = new btCapsuleShape(btScalar(0.025), btScalar(0.5));
	g_collisionshapes.push_back(capsule_shape); // 最後に破棄(delete)するために形状データを格納しておく

	btCollisionShape* c_zcylinder_shape = new btCylinderShapeZ(btVector3(0.115, -0.4, -0.4));

	btCompoundShape* compound = new btCompoundShape;

	btQuaternion qrot(0, 0, 0, 1);
	//車体の追加
	trans.setOrigin(btVector3(0, GROUND_HEIGHT, 0));
	trans.setRotation(qrot);// 四元数を行列に変換して姿勢行列に掛け合わせる
	compound->addChildShape(trans, box_shape);
	//車体の上の小さな段差
	trans.setOrigin(btVector3(0, GROUND_HEIGHT+CUBE_HALF_EXTENTS,0.2));
	trans.setRotation(qrot);
	compound->addChildShape(trans, c_box_shape);
	//旗の棒部分
	trans.setOrigin(btVector3(0.3, GROUND_HEIGHT + CUBE_HALF_EXTENTS+0.2, -0.7));
	trans.setRotation(qrot);
	compound->addChildShape(trans, capsule_shape);
	//砲台部分
	trans.setOrigin(btVector3(0, GROUND_HEIGHT + 0.1 + CUBE_HALF_EXTENTS * 1.5, 0.2));
	trans.setRotation(qrot);
	compound->addChildShape(trans, c_cylinder_shape);
	compound->recalculateLocalAabb();
	trans.setIdentity();
	trans.setOrigin(btVector3(0, GROUND_HEIGHT + 0.4, 0));
	btRigidBody* body1 = CreateRigidBody(1.0, trans, compound, g_dynamicsworld, 259);
	g_tracerbody = body1;

	//弾の射出部分
	trans.setOrigin(btVector3(0, GROUND_HEIGHT + CUBE_HALF_EXTENTS * 1.5+0.4 , -0.1));
	trans.setRotation(qrot);
	btRigidBody* body1_2 = CreateRigidBody(1.0, trans, c_zcylinder_shape, g_dynamicsworld, 259);
	g_ballpositionbody = body1_2;

	//二つの形状の連結
	btHingeConstraint* joint_c = new btHingeConstraint(*body1, *body1_2, btVector3(0, GROUND_HEIGHT + CUBE_HALF_EXTENTS * 1.5 +0.1, 0.2), btVector3(0.0, 0.0, 0.2), btVector3(0.0, 1.0, 0.0), btVector3(0.0, 1.0, 0.0));
	joint_c->enableAngularMotor(true, btRadians(0.0), 1.0);

	g_constraint_c = joint_c;
	g_dynamicsworld->addConstraint(joint_c);

	//二次元弾性体の追加
	btScalar sl1 = 0.3;
	btScalar sl2 = 0.7;
	btScalar y1 = GROUND_HEIGHT+CUBE_HALF_EXTENTS+0.6;
	btScalar y2 = GROUND_HEIGHT + CUBE_HALF_EXTENTS + 0.85;
	btScalar z = -0.7;
	int res = 9;
	btSoftBody* cloth = btSoftBodyHelpers::CreatePatch(g_softBodyWorldInfo,
		btVector3(sl1, y1, z), btVector3(sl1, y2, z), btVector3(sl2, y1, z), btVector3(sl2, y2, z),
		res, res,//分割数
		0, true);
	cloth->getCollisionShape()->setMargin(0.01);
	cloth->setTotalMass(0.02);
	cloth->m_materials[0]->m_kLST = 0.7;
	//Lift,Drag Fprceのための係数
	cloth->m_cfg.kLF = 0.05;
	cloth->m_cfg.kDG = 0.01;
	cloth->m_cfg.piterations = 2;//ばねによる位置修正の最大反復回数
	cloth->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;

	cloth->setWindVelocity(btVector3(40.0, -15.0, 0.0));

	g_dynamicsworld->addSoftBody(cloth);
	for (int i = 0; i < 9; i++) {
		cloth->appendAnchor(i, body1);
	}

	btCollisionShape* x_cylinder_shape = new btCylinderShapeX(btVector3(0.1, 0.2, 0.2));
	g_collisionshapes.push_back(x_cylinder_shape);
	trans.setOrigin(btVector3(2*CUBE_HALF_EXTENTS+0.1, GROUND_HEIGHT + 0.2, 3*CUBE_HALF_EXTENTS));//位置+,+ 
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる
	btRigidBody *body2 = CreateRigidBody(10.0, trans, x_cylinder_shape, g_dynamicsworld, 259);

	trans.setOrigin(btVector3(2 * CUBE_HALF_EXTENTS + 0.1, GROUND_HEIGHT + 0.2, -3 * CUBE_HALF_EXTENTS));//位置+,- ハンドルR
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる
	btRigidBody *body3 = CreateRigidBody(10.0, trans, x_cylinder_shape, g_dynamicsworld, 259);
	
	trans.setOrigin(btVector3(-2 * CUBE_HALF_EXTENTS - 0.1, GROUND_HEIGHT + 0.2, 3 * CUBE_HALF_EXTENTS));//位置-,+ 
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる
	btRigidBody *body4 = CreateRigidBody(10.0, trans, x_cylinder_shape, g_dynamicsworld, 259);
	
	trans.setOrigin(btVector3(-2 * CUBE_HALF_EXTENTS - 0.1, GROUND_HEIGHT + 0.2, -3 * CUBE_HALF_EXTENTS));//位置-,- ハンドルL
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる
	btRigidBody *body5 = CreateRigidBody(10.0, trans, x_cylinder_shape, g_dynamicsworld, 259);

	double tire_friction = 1.0;
	body2->setFriction(tire_friction);
	body3->setFriction(tire_friction);
	body4->setFriction(tire_friction);
	body5->setFriction(tire_friction);

	btHingeConstraint* joint12 = new btHingeConstraint(*body1, *body2, btVector3(2 * CUBE_HALF_EXTENTS, -0.2, 3 * CUBE_HALF_EXTENTS), btVector3(-0.1, 0.0, 0.0), btVector3(-1.0, 0.0, 0.0), btVector3(-1.0, 0.0, 0.0));
	btHinge2Constraint* joint13 = new btHinge2Constraint(*body1, *body3, btVector3(2 * CUBE_HALF_EXTENTS, GROUND_HEIGHT+0.2, -3 * CUBE_HALF_EXTENTS), btVector3(0.0, 1.0, 0.0), btVector3(1.0, 0.0, 0.0));
	btHingeConstraint* joint14 = new btHingeConstraint(*body1, *body4, btVector3(-2 * CUBE_HALF_EXTENTS, -0.2, 3 * CUBE_HALF_EXTENTS), btVector3(0.1, 0.0, 0.0), btVector3(1.0, 0.0, 0.0), btVector3(1.0, 0.0, 0.0), true);
	btHinge2Constraint* joint15 = new btHinge2Constraint(*body1, *body5, btVector3(-2 * CUBE_HALF_EXTENTS, GROUND_HEIGHT+0.2, -3 * CUBE_HALF_EXTENTS),btVector3(0.0, 1.0, 0.0), btVector3(1.0, 0.0, 0.0));
	
	joint12->enableAngularMotor(true, btRadians(0.0), 1.0);
	joint14->enableAngularMotor(true, btRadians(0.0), 1.0);
	joint13->setLowerLimit(btScalar(-1.0));
	joint13->setUpperLimit(btScalar(1.0));
	joint13->setLinearLowerLimit(btVector3(0, 0,0));
	joint13->setLinearUpperLimit(btVector3(0,0,0));
	joint15->setLowerLimit(btScalar(-1.0));
	joint15->setUpperLimit(btScalar(1.0));
	joint15->setLinearLowerLimit(btVector3(0, 0, 0));
	joint15->setLinearUpperLimit(btVector3(0, 0, 0));
	btRotationalLimitMotor2* motor1;
	btRotationalLimitMotor2* motor12;
	btRotationalLimitMotor2* motor2;
	btRotationalLimitMotor2* motor22;
	motor1 = joint13->getRotationalLimitMotor(0);
	motor12 = joint13->getRotationalLimitMotor(2);
	motor1->m_enableMotor = true;
	motor12->m_enableMotor = true;
	motor2 = joint15->getRotationalLimitMotor(0);
	motor22 = joint15->getRotationalLimitMotor(2);
	motor2->m_enableMotor = true;
	motor22->m_enableMotor = true;

	motor1->m_targetVelocity = btRadians(0.0);
	motor12->m_targetVelocity = 0;

	g_constraint1 = joint12;
	g_constraint2 = joint13;
	g_constraint3 = joint14;
	g_constraint4 = joint15;
	g_motor1 = motor1;
	g_motor2 = motor2;
	g_motor12 = motor12;
	g_motor22 = motor22;
	g_dynamicsworld->addConstraint(joint12);
	g_dynamicsworld->addConstraint(joint13);
	g_dynamicsworld->addConstraint(joint14);
	g_dynamicsworld->addConstraint(joint15);
	// ----- ここまで (立方体オブジェクト追加) -----

	// すり抜け防止用Swept sphereの設定(CCD:Continuous Collision Detection)
	body1->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
	body1->setCcdSweptSphereRadius(0.05*CUBE_HALF_EXTENTS);

	//障害物の追加
	btCollisionShape* obtacle_shape = new btBoxShape(btVector3(2.5, 5, 2.5 ));
	g_collisionshapes.push_back(obtacle_shape);
	trans.setIdentity();
	trans.setOrigin(btVector3(10, 5, 10));
	trans.setRotation(qrot);
	btRigidBody* obtacle_1 = CreateRigidBody(0.0, trans, obtacle_shape, g_dynamicsworld, 99);//右手前
	trans.setOrigin(btVector3(10, 5, -10));
	trans.setRotation(qrot);
	btRigidBody* obtacle_2 = CreateRigidBody(0.0, trans, obtacle_shape, g_dynamicsworld, 99);//右奥
	trans.setOrigin(btVector3(-10, 5, 10));
	trans.setRotation(qrot);
	btRigidBody* obtacle_3 = CreateRigidBody(0.0, trans, obtacle_shape, g_dynamicsworld, 99);//左手前
	trans.setOrigin(btVector3(-10, 5, -10));
	trans.setRotation(qrot);
	btRigidBody* obtacle_4 = CreateRigidBody(0.0, trans, obtacle_shape, g_dynamicsworld, 99);//左奥

	g_enemybody_1=CreateEnemyBody(10, btVector3(-15.75, 0.5, 15.75), btQuaternion(btRadians(0), 0, 0).normalize());//敵オブジェクトの初期位置
	g_enemybody_1->setLinearVelocity(btVector3(0, 0, -g_enemyspeed));

	//g_enemybody_1 = CreateEnemyBody(10, btVector3(0, 0.5, -10), btQuaternion(btRadians(180), 0, 0).normalize());//敵オブジェクトを正面に配置(上とどっちかのみ)

}

/*!
* Bullet初期化
*/
void InitBullet(void)
{
	// 衝突検出方法の選択(デフォルトを選択)
	btDefaultCollisionConfiguration *config = new btSoftBodyRigidBodyCollisionConfiguration();
	btCollisionDispatcher *dispatcher = new btCollisionDispatcher(config);

	// ブロードフェーズ法の設定(Dynamic AABB tree method)
	btAxisSweep3* broadphase = new btAxisSweep3(btVector3(-100, -100, -100), btVector3(100, 100, 100));

	// 拘束(剛体間リンク)のソルバ設定
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	// Bulletのワールド作成
	g_dynamicsworld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, config, 0);

	// 重力加速度の設定(OpenGLに合わせてy軸方向を上下方向にする)
	g_dynamicsworld->setGravity(btVector3(0, -9.8, 0));

	// btSoftBodyWorldInfoの初期化・設定
	g_softBodyWorldInfo.m_dispatcher = dispatcher;
	g_softBodyWorldInfo.m_broadphase = broadphase;
	g_softBodyWorldInfo.m_sparsesdf.Initialize();
	g_softBodyWorldInfo.m_gravity.setValue(0, -9.8, 0);
	g_softBodyWorldInfo.air_density = 1.2;
	g_softBodyWorldInfo.m_sparsesdf.Reset();

	SetRigidBodies();
}

void Square2D(int w,int h,int x1, int x2, int y1, int y2) {//Minimapのための四角形描画
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glRasterPos2f(x1, y1);

	glBegin(GL_QUADS);
	glVertex2i(x1, y1);
	glVertex2i(x2, y1);
	glVertex2i(x2, y2);
	glVertex2i(x1, y2);
	glEnd();

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void Circle2D(int w, int h,int x,int y) {//Minimapのための円の描画
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glRasterPos2f(x-15, y-15);

	int radius = 14;
	double PI = glm::pi<double>();
	for (float th1 = 0.0; th1 <= 360.0; th1 = th1 + 1.0)
	{
		float th2 = th1 + 10.0;
		float th1_rad = th1 / 180.0 * PI;
		float th2_rad = th2 / 180.0 * PI;

		float x1 = radius * cos(th1_rad);
		float y1 = radius * sin(th1_rad);
		float x2 = radius * cos(th2_rad);
		float y2 = radius * sin(th2_rad);

		glBegin(GL_TRIANGLES);
		glVertex2f(x, y);
		glVertex2f(x1 + x, y1 + y);
		glVertex2f(x2 + x, y2 + y);
		glEnd();
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void traceObject(btRigidBody* body,int w, int h, int x0, int y0, int color = 0) {//オブジェクトの位置と種類によってMinimap上での描画位置を変化
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 pos = trans.getOrigin();

	if (color == 0) {
		glColor4f(0.8, 0.0, 0.0, 1.0);//敵オブジェクトは赤色で表示
	}
	else if(color==1){
		glColor4f(0.0, 0.0, 0.8, 1.0);//操作オブジェクトは青色で表示
	}
	else {
		glColor4f(1.0, 0.50, 0.31, 1.0);//敵特殊オブジェクトはオレンジ色で表示
	}
	int x_pos, z_pos;

	if (pos[0] < -12.5) {
		x_pos = 20;
	}
	else if (pos[0] >= -12.5 && pos[0] < -7.5) {
		x_pos = 50;
	}
	else if (pos[0] >= -7.5 && pos[0] < -2.5) {
		x_pos = 80;
	}
	else if (pos[0] >= -2.5 && pos[0] < 2.5) {
		x_pos = 110;
	}
	else if (pos[0] >= 2.5 && pos[0] < 7.5) {
		x_pos = 140;
	}
	else if (pos[0] >= 7.5 && pos[0] < 12.5) {
		x_pos = 170;
	}
	else if (pos[0] >= 12.5){
		x_pos = 200;
	}

	if (pos[2] < -12.5) {
		z_pos = 20;
	}
	else if (pos[2] >= -12.5 && pos[2] < -7.5) {
		z_pos = 50;
	}
	else if (pos[2] >= -7.5 && pos[2] < -2.5) {
		z_pos = 80;
	}
	else if (pos[2] >= -2.5 && pos[2] < 2.5) {
		z_pos = 110;
	}
	else if (pos[2] >= 2.5 && pos[2] < 7.5) {
		z_pos = 140;
	}
	else if (pos[2] >= 7.5 && pos[2] < 12.5) {
		z_pos = 170;
	}
	else if (pos[2] >= 12.5) {
		z_pos = 200;
	}

	Circle2D(w, h, x0 + x_pos, y0 + z_pos);
}

void DrawLine(int w, int h, int x1, int y1,int x2,int y2) {//MiniMapのための線描画
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glRasterPos2f(x1-1, y1-1);

	glLineWidth(1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2f(x1, y1);
	glVertex2f(x2, y2);
	glEnd();

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void MiniMap(int w, int h, int x0, int y0) {//Minimapを描画
	glColor4f(1.0, 0.0, 0.0, 1.0);
	for (int i=0; i < 8; i++) {
		DrawLine(g_winw, g_winh, x0 + 5 + i * 30, y0, x0 + 5 + i * 30, y0 + 220);//区画わけの縦の線
	}

	for (int i=0; i < 8; i++) {
		DrawLine(g_winw, g_winh, x0, y0 + 5 + i * 30, x0 + 220, y0 + 5 + i * 30);//区画わけの横の線
	}

	glColor4f(0.1, 0.1, 0.1, 1.0);
	Square2D(w, h, x0 + 35, x0 + 65, y0 + 35, y0 + 65);//左上の障害物

	glColor4f(0.1, 0.1, 0.1, 1.0);
	Square2D(w, h, x0 + 35, x0 + 65, y0 + 155, y0 + 185);//左下の障害物

	glColor4f(0.1, 0.1, 0.1, 1.0);
	Square2D(w, h, x0 + 155, x0 + 185, y0 + 35, y0 + 65);//右上の障害物

	glColor4f(0.1, 0.1, 0.1, 1.0);
	Square2D(w, h, x0 + 155, x0 + 185, y0 + 155, y0 + 185);//右下の障害物

	traceObject(g_tracerbody, w, h, x0, y0, 1);//操作オブジェクト

	if (g_enemybody_1) {
		traceObject(g_enemybody_1, w, h, x0, y0, 0);//敵オブジェクト1
	}
	if (g_enemybody_2) {
		traceObject(g_enemybody_2, w, h, x0, y0, 0);//敵オブジェクト2
	}
	if (g_enemybody_3) {
		traceObject(g_enemybody_3, w, h, x0, y0, 2);//敵オブジェクト3
	}

	glColor4f(0.3, 0.75, 0.1, 0.8);
	Square2D(w, h, x0 + 5, x0 + 215, y0 + 5, y0 + 215);//マップの地面

	glColor4f(0.3, 0.3, 0.3, 0.4);
	Square2D(w,h,x0, x0 + 220, y0, y0 + 220);//マップの表示の下地

}


/*!
* 設定したBulletの剛体オブジェクト，ワールドの破棄
*/
void CleanBullet(void)
{
	// 剛体オブジェクトの破棄
	for (int i = g_dynamicsworld->getNumCollisionObjects() - 1; i >= 0; --i) {
		btCollisionObject* obj = g_dynamicsworld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}

		//オブジェクトがSoftBodyの場合の破棄
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody) {
			static_cast<btSoftRigidDynamicsWorld*>(g_dynamicsworld)->removeSoftBody(softBody);
		}
		else {
			static_cast<btSoftRigidDynamicsWorld*>(g_dynamicsworld)->removeCollisionObject(obj);
		}
		g_dynamicsworld->removeCollisionObject(obj);
		delete obj;
	}

	// 形状の破棄
	for (int j = 0; j < (int)g_collisionshapes.size(); ++j) {
		btCollisionShape* shape = g_collisionshapes[j];
		g_collisionshapes[j] = 0;
		delete shape;
	}
	g_collisionshapes.clear();

	// ワールド破棄
	delete g_dynamicsworld->getBroadphase();
	delete g_dynamicsworld;
}



//-----------------------------------------------------------------------------
// アプリケーション制御関数
//-----------------------------------------------------------------------------
/*!
* アニメーションN/OFF
* @param[in] on trueでON, falseでOFF
*/
bool switchanimation(int on)
{
	g_animation_on = (on == -1) ? !g_animation_on : (on ? true : false);
	return g_animation_on;
}

/*!
* 現在の画面描画を画像ファイルとして保存(連番)
* @param[in] stp 現在のステップ数(ファイル名として使用)
*/
void savedisplay(const int &stp)
{
	static int nsave = 1;
	string fn = CreateFileName("img_", ".bmp", (stp == -1 ? nsave++ : stp), 5);
	saveFrameBuffer(fn, g_winw, g_winh);
	std::cout << "saved the screen image to " << fn << std::endl;
}
/*!
* 視点の初期化
*/
void resetview(void)
{
	double q[4] = { 1, 0, 0, 0 };
	g_view.SetQuaternion(q);
	//g_view.SetRotation(20.0, 1.0, 0.0, 0.0);
	g_view.SetScaling(-7.0);
	g_view.SetTranslation(0.0, -2.0);
}
/*!
* シミュレーションのリセット
*/
void reset(void)
{
	CleanBullet();
	InitBullet();
	g_currentstep = 0;
	g_points = 0;   
	g_life = 3;
	g_targetsteerangle = 0;
	g_targetvelocity = 0;
	g_firingangle = 0;
	g_clear_flag = true;//クリアフラグ
	g_over_flag = false;//オーバーフラグ
	g_back_view = false;//視点を背後に固定
	g_end_flag = false;
	g_wait_time = 0;//発射可能な時間までの待ち時間
	g_spawntime = 0;
	//敵オブジェクト1の管理に利用
	g_turnflag = false;
	g_step = 0;
	g_tmp = 2;
	//敵オブジェクト2
	g_turnflag2 = false;
	g_step2 = 0;
	g_tmp2 = 2;
	//敵オブジェクト2
	g_turnflag3 = false;
	g_step3 = 0;
	g_tmp3 = 2;
	//ダメージの際の文字列描画に利用
	g_str_tmp = 0;
	g_str_tmp_flag = false;
	g_str_flag = false;
	//ポイントの際の文字列表示に利用
	g_str_tmp2 = 0;
	g_str_tmp_flag2 = false;
	g_str_flag2 = false;
	//ゲーム開始
	g_start_flag = true;

	if (g_enemybody_2)
		g_dynamicsworld->removeCollisionObject(g_enemybody_2);
	if (g_enemybody_3)
		g_dynamicsworld->removeCollisionObject(g_enemybody_3);

	//0に初期化
	btRigidBody* g_enemybody_1=0;
	btRigidBody* g_enemybody_2=0;
	btRigidBody* g_enemybody_3=0;

	resetview();
}


/*!
* 初期化関数
*/
void Init(void)
{
	// OpenGLのバージョンチェック
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	cout << "Vendor: " << glGetString(GL_VENDOR) << endl;
	cout << "Renderer: " << glGetString(GL_RENDERER) << endl;

	// GLEWの初期化
	GLenum err = glewInit();
	if(err != GLEW_OK) cout << "GLEW Error : " << glewGetErrorString(err) << endl;

	// 描画系フラグ設定(アンチエイリアス,デプステスト,隠面除去,法線計算,点描画)
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_POINT_SMOOTH);

	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_POSITION, glm::value_ptr(LIGHT0_POS));
	glLightfv(GL_LIGHT0, GL_DIFFUSE, glm::value_ptr(LIGHT_DIFF));
	glLightfv(GL_LIGHT0, GL_SPECULAR, glm::value_ptr(LIGHT_SPEC));
	glLightfv(GL_LIGHT0, GL_AMBIENT, glm::value_ptr(LIGHT_AMBI));

	// 視点初期化
	resetview();
	
	// シャドウマップ初期化
	g_shadowmap.InitShadow(g_shadowmap_res, g_shadowmap_res);

	// Bullet初期化
	InitBullet();

	switchanimation(1);
}


//-----------------------------------------------------------------------------
// OpenGL/GLFWコールバック関数
//-----------------------------------------------------------------------------
/*!
* Bulletのオブジェクトの描画シーン描画
* @param[in] x クラスのメンバ関数(static)を渡すときに用いるポインタ(グローバル関数の場合は使わないので0でOK)
*/
void DrawBulletObjects(void* x = 0)
{
	static const GLfloat difr[] = { 1.0, 0.4, 0.4, 1.0 };	// 拡散色 : 赤
	static const GLfloat difg[] = { 0.4, 0.6, 0.4, 1.0 };	// 拡散色 : 緑
	static const GLfloat difb[] = { 0.4, 0.4, 1.0, 1.0 };	// 拡散色 : 青
	static const GLfloat difo[] = { 1.0,0.50, 0.31,1.0 };   //オレンジ色
	static const GLfloat spec[] = { 0.4, 0.4, 0.4, 1.0 };	// 鏡面反射色
	static const GLfloat ambi[] = { 0.3, 0.3, 0.3, 1.0 };	// 環境光

	glDisable(GL_COLOR_MATERIAL);

	// 光源/材質設定
	glLightfv(GL_LIGHT0, GL_POSITION, glm::value_ptr(LIGHT0_POS));
	glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
	glMaterialfv(GL_FRONT, GL_AMBIENT,  ambi);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.0, 0.0, 1.0);

	glEnable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);

	if(g_dynamicsworld){
		btScalar m[16];
		btMatrix3x3	rot;
		rot.setIdentity();

		// Bulletワールドから剛体オブジェクト情報を取得してOpenGLで描画
		const int n = g_dynamicsworld->getNumCollisionObjects();	// オブジェクト数の取得
		for(int i = 0; i < n; ++i){

			// btCollisionObject → btRigidBodyへのキャストで剛体オブジェクトを取得
			btCollisionObject* obj = g_dynamicsworld->getCollisionObjectArray()[i];

			// 形状取得
			btCollisionShape* shape = obj->getCollisionShape();
			int shapetype = shape->getShapeType();

			if(shapetype == SOFTBODY_SHAPE_PROXYTYPE){
				btSoftBody* body = btSoftBody::upcast(obj);

				glMaterialfv(GL_FRONT, GL_DIFFUSE, difb);

				if (body->isActive()) {
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difr);
				}

				// draw a softbody
				DrawBulletSoftBody(body);
			}
			else{
				btRigidBody* body = btRigidBody::upcast(obj);
				if(body && body->getMotionState()){
					// btRigidBodyからMotion Stateを取得して，OpenGLの変換行列として位置・姿勢情報を得る
					btDefaultMotionState* ms = (btDefaultMotionState*)body->getMotionState();
					ms->m_graphicsWorldTrans.getOpenGLMatrix(m);
					rot = ms->m_graphicsWorldTrans.getBasis();
				}
				else{
					obj->getWorldTransform().getOpenGLMatrix(m);
					rot = obj->getWorldTransform().getBasis();
				}

				if(body && body->getInvMass() > 1e-6){
					// Dynamicボディは青で描画
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difb);
				}
				else {	// Kinematicボディの場合は緑で描画
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difg);
				}

				if (body->getUserIndex() == 10||body->getUserIndex()==200) {//敵オブジェクト、その弾を赤色で表示
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difr);
				}

				if (body->getUserIndex() == 15 || body->getUserIndex() == 250) {//敵特殊オブジェクト、その弾をオレンジ色で表示
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difo);
				}

				
				btVector3 world_min, world_max;
				g_dynamicsworld->getBroadphase()->getBroadphaseAabb(world_min, world_max);

				glPushMatrix();
#ifdef BT_USE_DOUBLE_PRECISION
				glMultMatrixd(m);
#else
				glMultMatrixf(m);
#endif

				// 形状描画
				DrawBulletShape(shape, world_min, world_max);

				glPopMatrix();
			}
		}
	}
}

void MoveCameraWithRigid(rxTrackball &view,btRigidBody *body) {//視点を車体の後ろに設定
	btVector3 r = btVector3(0, 2, 7);

	//剛体の位置と姿勢
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 p = trans.getOrigin();
	btQuaternion q = trans.getRotation();
	q = q.inverse();

	//視線方向を回転
	double qd[4];
	qd[0] = q[3]; qd[1] = q[0]; qd[2] = q[1]; qd[3] = q[2];
	view.SetQuaternion(qd);

	//剛体の後ろに視点を移動
	btVector3 epos = quatRotate(q, p) + r;
	view.SetTranslation(-epos[0], -epos[1]);
	view.SetScaling(-epos[2]);
}

void MoveCameraY(rxTrackball& view, btRigidBody* body) {//視点を俯瞰できる位置(r)に固定
	btVector3 r = btVector3(0, -10 , 50);

	//剛体の位置と姿勢
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 p = trans.getOrigin();
	btQuaternion q = trans.getRotation();
	q = q.inverse();

	//視線方向を回転
	double qd[4];
	qd[0] = btRadians(-30); qd[1] = btRadians(-30); qd[2] = 0; qd[3] = 0;
	view.SetQuaternion(qd);

	//剛体の後ろに視点を移動
	//btVector3 epos = quatRotate(q, p) + r;
	btVector3 epos = r;
	view.SetTranslation(-epos[0], -epos[1]);
	view.SetScaling(-epos[2]);
}

/*!
* 再描画イベントコールバック関数
*/
void Display(void)
{
	static bool point_flag = false;

	// ビューポート,透視変換行列,モデルビュー変換行列の設定
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glm::mat4 mp = glm::perspective(FOV, (float)g_winw/g_winh, 0.2f, 1000.0f);
	glMultMatrixf(glm::value_ptr(mp));
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// 描画バッファのクリア
	glClearColor((GLfloat)g_bgcolor[0], (GLfloat)g_bgcolor[1], (GLfloat)g_bgcolor[2], 1.0f);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();

	// マウスによる回転・平行移動の適用
	g_view.Apply();

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glDisable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);

	// 影なしでのオブジェクト描画
	//DrawBulletObjects();

	// シャドウマップを使って影付きでオブジェクト描画
	glm::vec3 light_pos(LIGHT0_POS[0], LIGHT0_POS[1], LIGHT0_POS[2]);
	ShadowMap::Frustum light = CalFrustum(80, 0.02, 20.0, g_shadowmap_res, g_shadowmap_res, light_pos, glm::vec3(0.0, -1.0, 0.0), glm::vec3(0.0, 1.0, 0.0));
	g_shadowmap.RenderSceneWithShadow(light, DrawBulletObjects, 0);

	//glDisable(GL_LIGHTING);

	int num_manifolds = g_dynamicsworld->getDispatcher()->getNumManifolds();//衝突候補のペアの数
	for (int i = 0; i < num_manifolds; ++i) {
		//衝突点を格納するためのキャッシュ(manifold)から情報を所得
		btPersistentManifold* manifold = g_dynamicsworld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = const_cast<btCollisionObject*>(manifold->getBody0());//衝突ペアのうちのオブジェクトA
		btCollisionObject* obB = const_cast<btCollisionObject*>(manifold->getBody1());//衝突ペアのうちのオブジェクトB

		//各オブジェクトのユーザーインデックス
		int user_idx0 = obA->getUserIndex();
		int user_idx1 = obB->getUserIndex();
		if (user_idx0 == 100){//obAが操作オブジェクトが発射した弾
			if (user_idx1 != 99) {
				if (user_idx1 == 10||user_idx1==15) {
					g_str_flag2 = true;
					g_str_tmp_flag2 = true;
					g_points++;
				}
					obA->setUserIndex(99);
					g_dynamicsworld->removeCollisionObject(obA);
					g_dynamicsworld->removeCollisionObject(obB);
					if (obB == g_enemybody_1)
						g_enemybody_1 = 0;
					if (obB == g_enemybody_2)
						g_enemybody_2 = 0;
					if (obB == g_enemybody_3)
						g_enemybody_3 = 0;
			}
		}
		else if (user_idx0 == 200||user_idx0==250) {//obAが敵オブジェクトの発射した弾
			if (user_idx1 == 259) {
				g_str_flag = true;
				g_str_tmp_flag = true;
				g_life--;
				obA->setUserIndex(99);
				g_dynamicsworld->removeCollisionObject(obA);
			}
		}

		if (user_idx1 == 100) {//obBが操作オブジェクトの発射した弾
			if (user_idx0 != 99) {
				if (user_idx0 == 10 || user_idx0 == 15) {
					g_str_flag2 = true;
					g_str_tmp_flag2 = true;
					g_points++;
				}
				
				obB->setUserIndex(99);
				g_dynamicsworld->removeCollisionObject(obA);
				g_dynamicsworld->removeCollisionObject(obB);
				if (obA == g_enemybody_1)
					g_enemybody_1 = 0;
				if (obA == g_enemybody_2)
					g_enemybody_2 = 0;
				if (obA == g_enemybody_3)
					g_enemybody_3 = 0;
			}
		}
		else if (user_idx1 == 200||user_idx1==250) {//obBが敵オブジェクトの発射した弾
			if (user_idx0 == 259) {
				g_str_flag = true;
				g_str_tmp_flag = true;
				g_life--;
				obB->setUserIndex(99);
				g_dynamicsworld->removeCollisionObject(obB);
			}
		}
	}
	
	if (g_points >= CLEAR) {//クリアの基準
		if (g_clear_flag) {
			//紙吹雪の実装
			int start = g_currentstep;
			btScalar sl = 0.15;
			int res = 3;

			btTransform trans2;
			g_tracerbody->getMotionState()->getWorldTransform(trans2);
			btVector3 p_body = trans2.getOrigin();
			int times = 0;
			for (times=0; times< 100; times++) {
				btScalar x = p_body[0] + 1 - (rand() % 200) / static_cast<btScalar>(100);
				btScalar y = p_body[1] + 4 - (rand() % 400) / static_cast<btScalar>(100);
				btScalar z = p_body[2] - (rand() % 200) / static_cast<btScalar>(100);
				btVector3 v;
				if (times % 4 == 0) {
					btVector3 v(-1, -1, -1);
				}
				else if (times % 4 == 1) {
					btVector3 v(-1, -1, 1);
				}
				else if (times % 4 == 2) {
					btVector3 v(-1, 1, 1);
				}
				else {
					btVector3 v(1, 1, 1);
				}

				btSoftBody* cloth = btSoftBodyHelpers::CreatePatch(g_softBodyWorldInfo,
					btVector3(x, y + 0.05 * v[0], z), btVector3(x, y - 0.05 * v[1], z + sl), btVector3(x + sl, y - 0.05 * v[2], z), btVector3(x + sl, y - 0.05, z + sl),
					res, res, 0, true);
				cloth->getCollisionShape()->setMargin(0.001);
				cloth->setTotalMass(0.00005);
				cloth->m_materials[0]->m_kLST = 0.05;
				//Lift,Drag Fprceのための係数
				cloth->m_cfg.kLF = 0.05;
				cloth->m_cfg.kDG = 0.01;
				cloth->m_cfg.piterations = 2;//ばねによる位置修正の最大反復回数
				cloth->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;

				cloth->setWindVelocity(btVector3(7.0 * v[2], 0.4, 7.0 * v[0]));
				cloth->addForce(btVector3(0.001 * (rand() % 3 - 1), 0.001 * (rand() % 3 - 1), 0.001 * (rand() % 3 - 1)));

				g_dynamicsworld->addSoftBody(cloth);
			}
			g_clear_flag = false;
		}
	}
	
	glPopMatrix();//境目-------------------------------------------------------------------------

	//ゲームスタート時の文字列
	vector<string> start_strs1 = { "Battle City" };
	vector<string> start_strs2 = { "press ENTER to start!!" };
	//常に表示するカウント部分
	vector<string> strs = { "your point is "+to_string(g_points)+"!","your life is "+to_string(g_life)};
	//クリア時に発生する文字列
	vector<string> clear_strs = { "you clear this game!!" };
	vector<string> clear_strs_sub = { "please press R if you wanna play again!" };
	//ゲームオーバー時に発生する文字列
	vector<string> over_strs = { "game over!!" };
	vector<string> over_strs2 = { "please press R to retly!" };
	//視点が固定されているときに発生する文字列
	vector<string> backview_strs = { "back view mode:ON","press I to quit back view mode" };

	DrawStrings(strs, g_winw, g_winh, 35, 180,20);//常に表示(左10,上30)
	
	//ダメージを受けた際に発生する文字列
	vector<string> damage_strs = { "you damaged!" };
	if (g_str_tmp_flag == true) {//ダメージを受けたときに文字列表示-----------------------------------
		g_str_tmp = g_currentstep;
		g_str_tmp_flag = false;
	}
	if (g_str_flag == true) {
		if (g_currentstep <= g_str_tmp + 120) {
			DrawStrings(damage_strs, g_winw, g_winh, 450, 480 + -(g_currentstep - g_str_tmp) / 2, 30);//480くらい
		}
		if (g_currentstep == g_str_tmp + 120) {
			g_str_flag = false;
		}
	}
	//--------------------------------------------------

	//ポイントを得た際に発生する文字列-------------------------------------------
	vector<string> point_strs = { "you pointed!" };
	if (g_str_tmp_flag2 == true) {//ダメージを受けたときに文字列表示-----------------------------------
		g_str_tmp2 = g_currentstep;
		g_str_tmp_flag2 = false;
	}
	if (g_str_flag2 == true) {
		if (g_currentstep <= g_str_tmp2 + 120) {
			DrawStrings(point_strs, g_winw, g_winh, 450, 300 + -(g_currentstep - g_str_tmp2) / 2, 30);//480くらい
		}
		if (g_currentstep == g_str_tmp2 + 120) {
			g_str_flag2 = false;
		}
	}
	//-------------------------------------------------------------
	if (g_start_flag) {
		DrawStrings(start_strs1, g_winw, g_winh, 360, 450, 60);
		DrawStrings(start_strs2, g_winw, g_winh, 320, 500, 45);
	}

	if (g_points >= CLEAR) {//クリアの基準
		DrawStrings(clear_strs, g_winw, g_winh, 320, 450,45);
		DrawStrings(clear_strs_sub, g_winw, g_winh, 300, 500, 25);
		g_end_flag = true;
	}
	btTransform trans2;
	g_tracerbody->getMotionState()->getWorldTransform(trans2);
	btVector3 p_body = trans2.getOrigin();
	if ((p_body[1] < -1||g_life<=0)&&g_points<CLEAR) {//ゲームオーバーの基準
		g_targetvelocity = 0;//ゲームオーバー時に操作オブジェクトを動かなくする
		g_over_flag = true;
		g_points = 0;
		DrawStrings(over_strs, g_winw, g_winh, 420, 500, 50);
		DrawStrings(over_strs2, g_winw, g_winh, 425, 530, 25);
		g_end_flag = true;
	}
	if (g_back_view) {
		DrawStrings(backview_strs, g_winw,g_winh,700, 512, 20);
	}
	if (g_end_flag) {//ゲーム終了時の処理
		if(g_enemybody_1)
			g_dynamicsworld->removeCollisionObject(g_enemybody_1);
		if(g_enemybody_2)
			g_dynamicsworld->removeCollisionObject(g_enemybody_2);
		if(g_enemybody_3)
			g_dynamicsworld->removeCollisionObject(g_enemybody_3);
		g_enemybody_1 = 0;
		g_enemybody_2 = 0;
		g_enemybody_3 = 0;
	}

	MiniMap(g_winw, g_winh, 800, 750);
}

void Enemy_Ball_fire(btRigidBody* body) {//敵オブジェクトの弾の発射間隔を設定
	btTransform trans;
	btVector3 ball_pos = Enemy_Ball_position(body);
	btVector3 ball_dir = Enemy_Ball_direction(body);
	btCollisionShape* sphere_shape = new btSphereShape(0.15);
	btQuaternion qrot(0, 0, 0, 1);
	btRigidBody* body1;
	
	if (body->getUserIndex() == 15) {//特別車体から発射
		if (g_currentstep % (ENEMYFIRE-200) == ENEMYFIRE - 251) {//発射間隔短め
			trans.setIdentity();// 位置姿勢行列の初期化
			trans.setOrigin(ball_pos);
			trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる

			body1 = CreateRigidBody(1.0, trans, sphere_shape, g_dynamicsworld, 200);//衝突判定を行うためにユーザインデックス200を設定
			body1->applyCentralImpulse(btVector3(ball_dir[0] * 50, ball_dir[1] * 50, ball_dir[2] * 50));
			body1->setCcdSweptSphereRadius(0.5);
			body1->setCcdMotionThreshold(0.05);
		}
	}
	else {
		if (g_currentstep % ENEMYFIRE == ENEMYFIRE - 1) {
			trans.setIdentity();// 位置姿勢行列の初期化
			trans.setOrigin(ball_pos);
			trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる

			body1 = CreateRigidBody(1.0, trans, sphere_shape, g_dynamicsworld, 200);//衝突判定を行うためにユーザインデックス200を設定
			body1->applyCentralImpulse(btVector3(ball_dir[0] * 40, ball_dir[1] * 40, ball_dir[2] * 40));
			body1->setCcdSweptSphereRadius(0.5);
			body1->setCcdMotionThreshold(0.05);
		}
	}
}

void CleanOutObject() {//不要なオブジェクトの削除を行う
	for (int i = g_dynamicsworld->getNumCollisionObjects() - 1; i >= 0; --i) {
		
		btCollisionObject* obj = g_dynamicsworld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		btTransform trans;
		trans=obj->getWorldTransform();
		btVector3 pos = trans.getOrigin();
		
		if (pos[1] < -1) {
			g_dynamicsworld->removeCollisionObject(obj);
			if (body == g_enemybody_1)
				g_enemybody_1 = 0;
			if (body == g_enemybody_2)
				g_enemybody_2 = 0;
			if (body == g_enemybody_3)
				g_enemybody_3 = 0;
		}
		
		btVector3 vel = obj->getInterpolationLinearVelocity();
		if (obj->getUserIndex() == 100) {//弾がフィールドに残った場合の処理
			if (abs(vel[0])<1&&abs(vel[1])<1&&abs(vel[2])<1)
				g_dynamicsworld->removeCollisionObject(obj);
		}
	}
}

/*!
* タイマーコールバック関数
*/
void Timer(void)
{
	if (g_start_flag) {
		g_animation_on = false;
	}

	if (g_animation_on) {
		if (g_dynamicsworld) {
			// シミュレーションを1ステップ進める
			g_dynamicsworld->stepSimulation(g_dt, 1);
		}
		g_currentstep++;
	}

	double gain = 0.4;
	//速度変更
	g_constraint1->setMotorTargetVelocity(btRadians(-g_targetvelocity));
	g_constraint3->setMotorTargetVelocity(btRadians(-g_targetvelocity));
	g_motor1->m_targetVelocity = btRadians(g_targetvelocity);
	g_motor2->m_targetVelocity = btRadians(g_targetvelocity);
	g_motor12->m_targetVelocity = gain * (btRadians(g_targetsteerangle)) - g_constraint2->getAngle1();
	g_motor22->m_targetVelocity = gain * (btRadians(g_targetsteerangle)) - g_constraint4->getAngle1();

	g_constraint_c->setLimit(btRadians(g_firingangle), btRadians(g_firingangle));

	//カメラを射撃方向に自動追従にする
	if(g_back_view)
		MoveCameraWithRigid(g_view, g_ballpositionbody);

	if (g_end_flag == false) {
		SpawnEnemyObject();//新規オブジェクトを生成
	}

	//-------------------------------------------------------------------------------------------------------------------------------------
	// 敵オブジェクトを出したくないときはコメントアウト
	//敵オブジェクト1の管理----------------------------------------------------------------------------------------
	if (g_enemybody_1) {
		if (enemypos(g_enemybody_1) == 1 && g_turnflag == false) {//回転処理を考慮する必要のある場所かどうかを判定
			g_turnflag = true;//回転処理中であるとき、true
			g_tmp = SetDirection(g_enemybody_1);//回転方向を判定
			g_step = g_currentstep;//回転処理を行うために、処理スタート時のステップを記憶
		}

		if (g_turnflag == true) {//回転処理中
			if (g_tmp == 0) {//反時計回り
				if (g_currentstep >= g_step && g_currentstep < g_step + 180) {//このステップの間回転を行う
					SetEnemydir(g_enemybody_1, 0);//回転を実行
				}
				else if (g_currentstep == g_step + 180) {//回転終了時
					g_enemybody_1->setAngularVelocity(btVector3(0, 0, 0));//回転速度を0に設定
					Enemyturn(g_enemybody_1, 0);//回転後の処理
				}
				else if (g_currentstep >= g_step + 210)//少し時間を空けて処理が再開できるようにする
					g_turnflag = false;

			}
			else if (g_tmp == 1) {
				if (g_currentstep >= g_step && g_currentstep < g_step + 180)
					SetEnemydir(g_enemybody_1, 1);
				else if (g_currentstep == g_step + 180) {
					Enemyturn(g_enemybody_1, 1);
					g_enemybody_1->setAngularVelocity(btVector3(0, 0, 0));
				}
				else if (g_currentstep == g_step + 210)
					g_turnflag = false;
			}
			else if (enemypos(g_enemybody_1) == 0) {
				if (g_currentstep > g_step + 50) {
					g_turnflag = false;
				}
			}
		}
		Enemy_Ball_fire(g_enemybody_1);
	}
	//--------------------------------------------------------------------------------------------------------------

	if (g_enemybody_2) {//敵オブジェクト2の管理-------------------------------------------------------------------
		if (enemypos(g_enemybody_2) == 1 && g_turnflag2 == false) {
			g_turnflag2 = true;
			g_tmp2 = SetDirection(g_enemybody_2);
			g_step2 = g_currentstep;
		}
		if (g_turnflag2 == true) {
			if (g_tmp2 == 0) {//反時計回り
				if (g_currentstep >= g_step2 && g_currentstep < g_step2 + 180)
					SetEnemydir(g_enemybody_2, 0);
				else if (g_currentstep == g_step2 + 180) {
					Enemyturn(g_enemybody_2, 0);
					g_enemybody_2->setAngularVelocity(btVector3(0, 0, 0));
				}
				else if (g_currentstep >= g_step2 + 210)
					g_turnflag2 = false;

			}
			else if (g_tmp2 == 1) {
				if (g_currentstep >= g_step2 && g_currentstep < g_step2 + 180)
					SetEnemydir(g_enemybody_2, 1);
				else if (g_currentstep == g_step2 + 180) {
					Enemyturn(g_enemybody_2, 1);
					g_enemybody_2->setAngularVelocity(btVector3(0, 0, 0));
				}
				else if (g_currentstep == g_step2 + 210)
					g_turnflag2 = false;
			}
			else if (enemypos(g_enemybody_2) == 0) {
				if (g_currentstep > g_step2 + 50) {
					g_turnflag2= false;
				}
			}
		}
		Enemy_Ball_fire(g_enemybody_2);
	}
	////--------------------------------------------------------------------------------------------------------------

	if (g_enemybody_3) {//敵オブジェクト3の管理-------------------------------------------------------------------
		if (enemypos(g_enemybody_3) == 1 && g_turnflag3 == false) {
			g_turnflag3 = true;
			g_tmp3 = SetDirection(g_enemybody_3);
			g_step3 = g_currentstep;
		}
		if (g_turnflag3 == true) {
			if (g_tmp3 == 0) {//反時計回り
				if (g_currentstep >= g_step3 && g_currentstep < g_step3 + 180)
					SetEnemydir(g_enemybody_3, 0);
				else if (g_currentstep == g_step3 + 180) {
					Enemyturn(g_enemybody_3, 0);
					g_enemybody_3->setAngularVelocity(btVector3(0, 0, 0));
				}
				else if (g_currentstep >= g_step3 + 210)
					g_turnflag3 = false;

			}
			else if (g_tmp3 == 1) {
				if (g_currentstep >= g_step3 && g_currentstep < g_step3 + 180)
					SetEnemydir(g_enemybody_3, 1);
				else if (g_currentstep == g_step3 + 180) {
					Enemyturn(g_enemybody_3, 1);
				}
				else if (g_currentstep == g_step3 + 210) {
					g_enemybody_3->setAngularVelocity(btVector3(0, 0, 0));
					g_turnflag3 = false;
				}
			}
			else if (enemypos(g_enemybody_3) == 0) {
				if (g_currentstep > g_step3 + 50) {
					g_turnflag3 = false;
				}
			}
		}
		Enemy_Ball_fire(g_enemybody_3);
	}
	//-------------------------------------------------------------------------------------------------------------------------------------
	g_wait_time++;//発射間隔の管理
	
	if (g_currentstep % 100 == 99) 
		CleanOutObject();
}


/*!
* キーボードイベント処理関数
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] key キーの種類 -> https://www.glfw.org/docs/latest/group__keys.html
* @param[in] scancode キーのスキャンコード(プラットフォーム依存)
* @param[in] action アクション(GLFW_PRESS:キーを押す, GLFW_RELESE:キーを離す，GLFW_REPEAT:キーリピート機能時)
* @param[in] mods 修飾キー(CTRL,SHIFT,ALT) -> https://www.glfw.org/docs/latest/group__mods.html
*/
void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	btTransform trans;	// 剛体オブジェクトの位置姿勢を格納する変数(行列)
	btTransform new_trans;
	trans.setIdentity();// 位置姿勢行列の初期化
	const btScalar CUBE_HALF_EXTENTS = 0.2;	// 立方体の変の長さの半分(中心から辺までの距離)
	const btScalar GROUND_HEIGHT = 0.0;		// 地面の高さ
	const btScalar SPHERE_RAD = 0.1; //球の半径、円柱の底面の半径
	const btScalar MOVE_DISTANCE = 0.02; //物体を矢印キーで動かす場合の幅
	btScalar MOVE_RAD = 0;
	double gain = 0.4;
	btCollisionShape* box_shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
	btCollisionShape* target_shape = new btBoxShape(btVector3(5*CUBE_HALF_EXTENTS, 5*CUBE_HALF_EXTENTS, 5*CUBE_HALF_EXTENTS));
	btCollisionShape* sphere_shape = new btSphereShape(SPHERE_RAD);
	btCollisionShape* cylinder_shape = new btCylinderShape(btVector3(SPHERE_RAD, CUBE_HALF_EXTENTS, SPHERE_RAD));
	btCollisionShape* brock_shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS, 5.0, CUBE_HALF_EXTENTS));
	glm::vec3 eye_pos(0.0), eye_dir(0.0);
	g_view.CalLocalPos(eye_pos, glm::vec3(0.0));
	g_view.CalLocalRot(eye_dir, glm::vec3(0.0, 0.0, -1.0));
	btVector3 ball_dir = Ball_direction(g_ballpositionbody,g_tracerbody);
	btVector3 ball_pos = Ball_position(g_ballpositionbody,g_tracerbody);
	btVector3 pos;
	btQuaternion q;

	btQuaternion qrot(0, 0, 0, 1);
	btRigidBody* body1;

	if(ImGui::GetIO().WantCaptureKeyboard) return;	// ImGUIウィンドウ上でのキーボードイベント時
	if(action == GLFW_PRESS || action == GLFW_REPEAT){
		switch(key){
		case GLFW_KEY_ESCAPE:	// ESC,Qキーでアプリケーション終了
		case GLFW_KEY_Q:
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;

		case GLFW_KEY_S: // SキーでアニメーションON/OFF
			switchanimation(-1);
			break;
		case GLFW_KEY_SPACE: // スペースキーでアニメーションを1ステップだけ進める
			g_animation_on = true; Timer(); g_animation_on = false;
			break;

		case GLFW_KEY_ENTER:
			g_start_flag = false;
			if (g_animation_on == false)
				g_animation_on = true;
			break;

		case GLFW_KEY_R: // Rキーでシーン(シミュレーション)リセット
			reset();
			break;
		
		case GLFW_KEY_O : //ライフを0にする(デバッグ用)
			g_life = 0;
			break;

		case GLFW_KEY_I: //Iキーで視点追従の有無を決める
			if (g_back_view)
				g_back_view = false;
			else
				g_back_view = true;
			
			break;

		case GLFW_KEY_RIGHT://タイヤを右方向に傾ける
			if (g_targetsteerangle < 120)
				g_targetsteerangle += 10.0;
			break;

		case GLFW_KEY_LEFT://タイヤを左方向に傾ける
			if (g_targetsteerangle>-120)
				g_targetsteerangle -= 10.0;
			break;

		case GLFW_KEY_UP://タイヤを前方向に回転させる
			if (g_targetvelocity < 0)
				g_targetvelocity += 15.0;
			g_targetvelocity += 15.0;
			break;

		case GLFW_KEY_DOWN://タイヤを後ろ方向に回転させる
			if (g_targetvelocity > 0)
				g_targetvelocity -= 15.0;
			g_targetvelocity -= 15.0;
			break;

		case GLFW_KEY_H://視点を車体の後ろに設定(固定はしない)
			MoveCameraWithRigid(g_view, g_tracerbody);
			break;

		case GLFW_KEY_Z://操作オブジェクトから弾を発射
			if (g_wait_time > FIRE) {//弾の発射間隔
				trans.setIdentity();// 位置姿勢行列の初期化
				trans.setOrigin(ball_pos);
				trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる

				body1 = CreateRigidBody(1.0, trans, sphere_shape, g_dynamicsworld, 100);//衝突判定を行うためにユーザインデックス100を設定
				body1->applyCentralImpulse(btVector3(ball_dir[0] * 50, ball_dir[1] * 50, ball_dir[2] * 50));
				body1->setCcdSweptSphereRadius(0.5);
				body1->setCcdMotionThreshold(0.05);
				g_wait_time = 0;
			}

			break;

		case GLFW_KEY_B://ブレーキの実装
			g_targetvelocity = 0;
			g_tracerbody->setLinearVelocity(btVector3(0, 0, 0));
			break;

		case GLFW_KEY_0://視点を俯瞰できる位置に移動
			MoveCameraY(g_view, g_tracerbody);
			break;

		case GLFW_KEY_A://弾の発射方向を反時計回りに動かす
			g_firingangle -= 1.0;
			break;

		case GLFW_KEY_D://弾の発射方向を時計回りに動かす
			g_firingangle += 1.0;
			break;

		case GLFW_KEY_Y://pointを増やす(デバッグ用)
			g_points++;
			break;

		case GLFW_KEY_1:
			g_firingangle = 0;
			break;

		case GLFW_KEY_2:
			g_firingangle = -90;
			break;

		case GLFW_KEY_3:
			g_firingangle = -180;
			break;

		case GLFW_KEY_4:
			g_firingangle = 90;
			break;

		default:
			break;
		}
	}
}

/*!
* マウスイベント処理関数
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] button マウスボタン(GLFW_MOUSE_BUTTON_LEFT,GLFW_MOUSE_BUTTON_MIDDLE,GLFW_MOUSE_BUTTON_RIGHT)
* @param[in] action マウスボタンの状態(GLFW_PRESS, GLFW_RELEASE)
* @param[in] mods 修飾キー(CTRL,SHIFT,ALT) -> https://www.glfw.org/docs/latest/group__mods.html
*/
void Mouse(GLFWwindow* window, int button, int action, int mods)
{
	if(ImGui::GetIO().WantCaptureMouse) return;	// ImGUIウィンドウ上でのマウスイベント時
	double x, y;
	glfwGetCursorPos(window, &x, &y);
	if(button == GLFW_MOUSE_BUTTON_LEFT){
		if(action == GLFW_PRESS){
			g_picknode = 0;
			glm::vec3 ray_from0, ray_to0;
			glm::vec3 init_pos(0, 0, 0);
			g_view.CalLocalPos(ray_from0, init_pos);
			g_view.GetRayTo(x, y, FOV, ray_to0);

			btVector3 ray_from = btVector3(ray_from0[0], ray_from0[1], ray_from0[2]);
			btVector3 ray_to = btVector3(ray_to0[0], ray_to0[1], ray_to0[2]);

			btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from, ray_to);
			g_dynamicsworld->rayTest(ray_from, ray_to, ray_callback);

			if(ray_callback.hasHit()){
				const btCollisionObject* obj = ray_callback.m_collisionObject;

				// 光線と衝突した剛体
				btRigidBody* body = const_cast<btRigidBody*>(btRigidBody::upcast(obj));

				// 衝突点座標(ジョイントになる位置座標)
				btVector3 picked_pos = ray_callback.m_hitPointWorld;

				if(body){
					if(!(body->isStaticObject() || body->isKinematicObject())){
						g_pickbody = body;
						g_pickpos = picked_pos;

						// 選択された剛体の座標系でのピック位置
						btVector3 local_pos = body->getCenterOfMassTransform().inverse()*picked_pos;

						g_pickbody->setActivationState(DISABLE_DEACTIVATION); // 必要！

						if(g_pickconstraint){
							g_dynamicsworld->removeConstraint(g_pickconstraint);
							delete g_pickconstraint;
						}
						g_pickconstraint = new btPoint2PointConstraint(*body, local_pos);
						g_dynamicsworld->addConstraint(g_pickconstraint, true);

						g_pickconstraint->m_setting.m_impulseClamp = 30.0;
						g_pickconstraint->m_setting.m_tau = 0.001f;

						g_pickdist = (g_pickpos-ray_from).length();
					}
				} 
				else{
					// 光線と衝突したbtSoftBody
					btSoftBody* body = const_cast<btSoftBody*>(btSoftBody::upcast(obj));
					btSoftBody::sRayCast res;
					body->rayTest(ray_from, ray_to, res);
					if(res.fraction < 1.0){
						btVector3 impact = ray_from+(ray_to-ray_from)*res.fraction;
						cout << impact << endl;
						if(res.feature == btSoftBody::eFeature::Face){
							btSoftBody::Face& face = res.body->m_faces[res.index];

							// 衝突点に最も近いノードを探索
							btSoftBody::Node* node = face.m_n[0];
							for(int i = 1; i < 3; ++i){
								if((node->m_x-impact).length2() >(face.m_n[i]->m_x-impact).length2()){
									node = face.m_n[i];
								}
							}
							g_picknode = node;
							g_pickdist = (g_picknode->m_x - ray_from).length();
						}
					}
				}
			}

			if(!g_pickconstraint && !g_picknode){
				// マウスドラッグによる視点移動
				g_view.Start(x, y, mods);
			}
		}
		else if(action == GLFW_RELEASE){
			if(g_pickconstraint){
				g_dynamicsworld->removeConstraint(g_pickconstraint);
				delete g_pickconstraint;
				g_pickconstraint = 0;
				g_pickbody = 0;
			}
			else if(g_picknode){
				g_picknode = 0;
			}
			else{
				g_view.Stop(x, y);
			}
		}
	}
}
/*!
* モーションイベント処理関数(マウスボタンを押したままドラッグ)
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] x,y マウス座標(スクリーン座標系)
*/
void Motion(GLFWwindow* window, double x, double y)
{
	if(ImGui::GetIO().WantCaptureMouse) return;	// ImGUIウィンドウ上でのマウスイベント時
	if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE &&
		glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_RELEASE &&
		glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE){
		return;
	}

	if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS){
		if(g_pickconstraint || g_picknode){
			glm::vec3 ray_from0, ray_to0;
			glm::vec3 init_pos(0, 0, 0);
			g_view.CalLocalPos(ray_from0, init_pos);
			g_view.GetRayTo(x, y, FOV, ray_to0);

			btVector3 ray_from = btVector3(ray_from0[0], ray_from0[1], ray_from0[2]);
			btVector3 new_ray_to = btVector3(ray_to0[0], ray_to0[1], ray_to0[2]);

			btVector3 dir = new_ray_to-ray_from;
			dir.normalize();

			btVector3 new_pivot = ray_from+dir*g_pickdist;

			if(g_pickconstraint){
				g_pickconstraint->setPivotB(new_pivot);
			}
			else if(g_picknode){
				//g_picknode->m_x = new_pivot;
				g_picknode->m_f += (new_pivot-g_picknode->m_x)*10.0;
			}

			g_pickpos = new_pivot;
		}
		else{
			g_view.Motion(x, y);
		}
	}
}

/*!
* リサイズイベント処理関数
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] w キャンバス幅(ピクセル数)
* @param[in] h キャンバス高さ(ピクセル数)
*/
void Resize(GLFWwindow* window, int w, int h)
{
	g_winw = w; g_winh = h;
	g_view.SetRegion(w, h);
	glViewport(0, 0, g_winw, g_winh);
}

/*!
* ImGUIのウィジット配置
*  - ImGUI/imgui_demo.cppを参考に ( https://github.com/ocornut/imgui#demo )
* @param[in] window コールバック関数を呼んだウィンドウハンドル
*/
void SetImGUI(GLFWwindow* window)
{
	ImGui::Text("simulation:");
	if(ImGui::Button("start/stop")){ switchanimation(-1); } ImGui::SameLine();
	if(ImGui::Button("run a step")){ g_animation_on = true; Timer(); g_animation_on = false; }
	if(ImGui::Button("reset")){ reset(); }
	ImGui::Separator();
	ImGui::InputFloat("dt", &(g_dt), 0.001f, 0.01f, "%.3f");
	ImGui::Separator();
	if(ImGui::Button("reset viewpos")){ resetview(); } 
	if(ImGui::Button("save screenshot")){ savedisplay(-1); }
	if(ImGui::Button("quit")){ glfwSetWindowShouldClose(window, GL_TRUE); }
	ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "Goal");
}

void Clean()
{
	CleanBullet();
}

/*!
 * メインルーチン
 * @param[in] argc コマンドライン引数の数
 * @param[in] argv コマンドライン引数
 */
int main(int argc, char *argv[])
{
	if(!glfwInit()) return 1;
	glfwSetErrorCallback(glfw_error_callback);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#ifdef __APPLE__
	glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
#endif

	// Create window
	GLFWwindow* window = glfwCreateWindow(g_winw, g_winh, "OpenGL Application", NULL, NULL);
	if(window == NULL) return 1;

	// Set glfw window as current OpenGL rendering context
	glfwMakeContextCurrent(window);
	glewExperimental = GL_TRUE;
	glfwSwapInterval(0); // Disable vsync

	// Initilization for OpenGL
	Init();

	// Setup callback functions
	glfwSetCursorPosCallback(window, Motion);
	glfwSetMouseButtonCallback(window, Mouse);
	glfwSetKeyCallback(window, Keyboard);
	glfwSetFramebufferSizeCallback(window, Resize);
	Resize(window, g_winw, g_winh);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL2_Init();

	// Settings for timer
	float cur_time = 0.0f, last_time = 0.0f, elapsed_time = 0.0f;
	glfwSetTime(0.0);	// Initialize the glfw timer

	// Main loop
	while(!glfwWindowShouldClose(window))
	{
		// Poll and handle events (inputs, window resize, etc.)
		glfwPollEvents();

		// OpenGL Rendering & Animation function
		Display();

		// Timer
		cur_time = glfwGetTime();
		elapsed_time = cur_time-last_time;
		if(elapsed_time >= g_dt){
			Timer();
			last_time = glfwGetTime();
		}

		// Start the ImGui frame
		ImGui_ImplOpenGL2_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// GUI
		ImGui::Begin("ImGui Window");
		ImGui::Text("Framerate: %.3f ms/frame (%.1f fps)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Separator();
		SetImGUI(window);
		ImGui::End();

		// Rendering of the ImGUI frame in opengl canvas
		ImGui::Render();
		ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
	}

	// Cleanup
	Clean();
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();


	return 0;
}