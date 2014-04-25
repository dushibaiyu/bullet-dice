/**
 * Copyright (c) 2011-2014 Microsoft Mobile and/or its subsidiary(-ies).
 * All rights reserved.
 *
 * OpenGL ES 2.0 and physics example.
 *
 * For the applicable distribution terms see the license.txt -file, included in
 * the distribution.
 */

#include <QtGui>
#include <QtOpenGL>
#include "PhysicsWidget.h"


// Define simple-cube for shadow-rendering.
#define SBS (0.0635f * DICE_SCALE)        // shadow-box-size

const GLfloat cube_vertices[] = {
    -SBS,-SBS,SBS,   SBS,-SBS,SBS,   SBS,SBS,SBS,   -SBS,SBS,SBS,
   -SBS,-SBS,-SBS,  SBS,-SBS,-SBS,  SBS,SBS,-SBS,  -SBS,SBS,-SBS
};


const GLushort cube_indices[] = {
    0,1,2,  0,2,3,  1,5,6,  1,6,2,  4,0,3,  4,3,7,
    5,4,7,  5,7,6,  4,5,1,  4,1,0,  2,6,7,  2,7,3
};

// Table defined as a single quad
const float table_vert[] = {
    -1.0f,-1.0f,0.0f,
    1.0f,-1.0f,0.0f,
    1.0f,1.0f,0.0f,
    -1.0f,1.0f,0.0f
};


const float table_uv[] = {
    0.0f,0.0f,
    1.0f,0.0f,
    1.0f,1.0f,
    0.0f,1.0f
};


const float table_normal[] = {
    0.0f,0.0f,1.0f,
    0.0f,0.0f,1.0f,
    0.0f,0.0f,1.0f,
    0.0f,0.0f,1.0f
};


PhysicsWidget::PhysicsWidget(QWidget *parent)
    : QGLWidget(parent)
{
    m_infoViewShown = false;
    m_mousePressed = false;
    m_viewDistance = 8.0f;
    m_tableAspectRatio = 800.0f / 424.0f;
    m_diceMesh = 0;

    setAutoFillBackground(false);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_PaintOnScreen, true);
    setAttribute(Qt::WA_StyledBackground, false);
    setAttribute(Qt::WA_AcceptTouchEvents);
    setAutoBufferSwap(false);

    // Load mesh from QRC.
    QFile qfile(":/dice.dig3d");

    if (qfile.open(QIODevice::ReadOnly)) {
        int filesize = qfile.size();
        char *rawData = new char[filesize];
        qfile.read(rawData, filesize);
        m_diceMesh = ExampleLoader::loadDig3D(rawData, filesize,
                                              5.0f * DICE_SCALE);
        delete [] rawData;
    }

    createWorld(INITIAL_CUBE_COUNT);

    m_infoText.setPerformanceHint(QStaticText::AggressiveCaching);
    m_infoText.setTextFormat(Qt::RichText);
    m_infoText.setText("<h1>BulletDice 1.1</h1>"
                       "<p>This is a Nokia Developer example demonstrating "
                       "the use of QtOpenGL with the Bullet Physics "
                       "Library. For more information about the project see "
                       "the web page: "
                       "https://projects.developer.nokia.com/gles2phys.</p>"
                       "<p>The Bullet Physics Library comes with ZLib "
                       "license:</p>"
                       "<p>Copyright (c) 2003-2010 Erwin Coumans "
                       "http://continuousphysics.com/Bullet/<br>"
                       "This software is provided 'as-is', without any "
                       "express or implied warranty. In no event will the "
                       "authors be held liable for any damages arising from "
                       "the use of this software. Permission is granted to "
                       "anyone to use this software for any purpose, "
                       "including commercial applications, and to alter it "
                       "and redistribute it freely, subject to the following "
                       "restrictions:</p>"
                       "<p>1. The origin of this software must not be "
                       "misrepresented; you must not claim that you wrote "
                       "the original software. If you use this software in a "
                       "product, an acknowledgment in the product "
                       "documentation would be appreciated but is not "
                       "required.</p>"
                       "<p>2. Altered source versions must be plainly marked "
                       "as such, and must not be misrepresented as being the "
                       "original software.</p>"
                       "<p>3. This notice may not be removed or altered from "
                       "any source distribution.</p>");


#if defined(Q_OS_SYMBIAN) || defined(Q_WS_HARMATTAN)
    m_Accelerometer = new QAccelerometer(this);
    m_Accelerometer->start();
#endif

    QTimer *timer = new QTimer(this);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(updateGL()));
    timer->start(16);
}


PhysicsWidget::~PhysicsWidget() {

    deleteWorld();

    // release the mesh(es)
    ExampleLoader::Mesh *mesh = m_diceMesh;

    while (mesh) {
        ExampleLoader::Mesh *next = mesh->nextMesh;
        delete mesh;
        mesh = next;
    }

    m_diceMesh = 0;
}


void PhysicsWidget::initializeGL ()
{
    m_infoButtonPixmap = new QPixmap(":/info_button.png");

#ifdef Q_OS_SYMBIAN
    m_closeButtonPixmap = new QPixmap(":/close_button.png");
#endif

    diceProgram = new QGLShaderProgram(context(), this);
    tableProgram = new QGLShaderProgram(context(), this);
    shadowProgram = new QGLShaderProgram(context(), this);

    glGenTextures(1, &m_combinedTexture);
    m_combinedTexture = bindTexture(QImage(":/combined.png"));
    glGenTextures(1, &m_tableTexture);
    m_tableTexture = bindTexture(QImage(":/table.png"));

    /**
     *	    SHADOW SHADER PROGRAM
     */
    QGLShader *shadowvshader = new QGLShader(QGLShader::Vertex, context(),
                                             this);
    const char *shadowvsrc =
            "attribute highp vec4 vertex;\n"
            "uniform mediump mat4 matrix;\n"
            "uniform mediump mat4 proj;\n"
            "varying lowp vec4 frag_pos;\n"
            "void main(void)\n"
            "{\n"
            "    highp vec4 temp_pos = matrix*vertex;\n"
            // create a vector from the light to the vertex and project it onto the Z0-plane
            "    highp vec3 ltov = normalize(vec3(temp_pos[0]+0.5, temp_pos[1]-0.5, temp_pos[2]+3.0));\n"
            "    highp float m = max( ((-8.0-temp_pos[2])/ltov[2]), 0.0);\n"
            "    frag_pos = temp_pos + vec4(ltov[0]*m, ltov[1]*m, ltov[2]*m, 0.0);\n"
            "    gl_Position = proj * frag_pos;\n"
            "}\n";

    QGLShader *shadowfshader = new QGLShader(QGLShader::Fragment, context(), this);
    const char *shadowfsrc =
            "varying highp vec4 frag_pos;\n"
            "uniform sampler2D tex;\n"
            "void main(void)\n"
            "{\n"
            // 0.9433962 = 0.5 * (800/424) -> due our aspectratio.
            "    mediump vec4 texpos = vec4( 0.5+frag_pos[0]*0.5, 0.5+frag_pos[1]*0.9433962, 0.0, 0.0);\n"
            "    mediump vec4 bg_col = (texture2D(tex, texpos.st).rgba);\n"
            //"    mediump float sm = 1.0 - bg_col.a*0.5;\n"
            "    gl_FragColor = bg_col*(1.0 - bg_col.a*0.5);\n"
            //"    gl_FragColor = vec4(bg_col[0]*sm, bg_col[1]*sm, bg_col[2]*sm, 1.0);\n"
            //"    gl_FragColor = vec4(1.0, 1.0, 0.0, 0.0);\n"
            "}\n";

    if (shadowvshader->compileSourceCode( shadowvsrc ) == false) {
        qDebug() << "Failed to compile shadow vertex shader";
    }

    if (shadowfshader->compileSourceCode( shadowfsrc ) == false) {
        qDebug() << "Failed to compile shadow fragment shader";
    }

    shadowProgram->addShader(shadowvshader);
    shadowProgram->addShader(shadowfshader);
    shadowProgram->link();
    shadowVertexAttr = shadowProgram->attributeLocation("vertex");
    shadowMatrixUniform = shadowProgram->uniformLocation("matrix");
    shadowMatrixProjection = shadowProgram->uniformLocation("proj");
    shadowTextureUniform = shadowProgram->uniformLocation("tex");

    /**
     *	    DICE SHADER PROGRAM
     */
    QGLShader *vshader1 = new QGLShader(QGLShader::Vertex, context(), this);
    const char *vsrc1 =
	    "attribute highp vec4 vertex;\n"
	    "attribute highp vec4 normal;\n"
	    "attribute highp vec4 texCoord;\n"
	    "uniform mediump mat4 matrix;\n"
	    "uniform mediump mat4 proj;\n"
	    "varying highp vec4 frag_normal;\n"
	    "varying highp vec4 frag_pos;\n"
	    "varying highp vec4 texc;\n"
	    "void main(void)\n"
	    "{\n"
	    "    texc = texCoord;\n"
	    "    frag_normal = matrix * vec4(normal[0], normal[1], normal[2], 0.0);\n"
	    "    frag_pos = matrix * vertex;\n"
	    "    gl_Position = proj * frag_pos;\n"
	    "}\n";


    if (vshader1->compileSourceCode(vsrc1) == false) {
        qDebug() << "Failed to compile dice vertex shader";
    }

    QGLShader *fshader1 = new QGLShader(QGLShader::Fragment, context(), this);

    const char *fsrc1 =
	    "varying highp vec4 frag_normal;\n"
	    "varying highp vec4 frag_pos;\n"
	    "varying highp vec4 texc;\n"
	    "uniform mediump vec3 matcol;\n"
	    "uniform sampler2D tex;\n"
	    "void main(void)\n"
	    "{\n"
            "    mediump vec4 toLight = vec4(-1.1, 1.1, -3.0, 0.0) - frag_pos;\n"
            // reflection color
            // 0.9433962 = 0.5 * (800/424) -> due our aspectratio.
            "	 mediump vec2 tet = vec2( 0.75+frag_pos[0]*0.25+frag_normal[0]*0.06,0.5+frag_pos[1]*0.9433962+frag_normal[1]*0.06);\n"
            "    mediump vec4 sampled_col = (texture2D(tex, tet).rgba);\n"
            // texturecolor
	    "	 tet = vec2( texc[0]*0.5,texc[1]);\n"
            "    mediump vec4 texture_col = (texture2D(tex, tet).rgba);\n"
            "    mediump float diffuse_mul = max(dot( normalize(frag_normal), normalize(toLight) ), 0.0);\n"
            // distance from light reduces diffuse light
	    "    diffuse_mul = diffuse_mul*3.0 / (toLight[0]*toLight[0]+toLight[1]*toLight[1]+2.0);\n"
            "    mediump vec3 col = vec3( (texture_col[0]*diffuse_mul)+sampled_col[0]*0.5, (texture_col[1]*diffuse_mul)+sampled_col[1]*0.5, (texture_col[2]*diffuse_mul)+sampled_col[2]*0.5) * matcol;\n"
            // specular light
            "    mediump float projtemp = dot(toLight, frag_normal) / dot(toLight,toLight);\n"
            "    mediump vec4 temp_vec = (toLight*projtemp);\n"
            "    temp_vec += (frag_normal - temp_vec)*2.0;\n"
            "    mediump float specular = max( (dot( normalize(temp_vec), normalize(toLight) )-0.66)*3.0, 0.0);\n"
	    "    col += vec3(1.0*specular, 1.0*specular, 1.0*specular);\n"
            "    gl_FragColor = vec4(clamp(col, 0.0, 1.0), 0.2);\n"
            //"    gl_FragColor = vec4(clamp(col, 0.0, 1.0), 0.2);\n"
	    "}\n";

    if (fshader1->compileSourceCode(fsrc1) == false) {
        qDebug() << "Failed to compile dice fragment shader";
    }

    diceProgram->addShader(vshader1);
    diceProgram->addShader(fshader1);
    diceProgram->link();
    vertexAttr1 = diceProgram->attributeLocation("vertex");
    texCoordAttr1 = diceProgram->attributeLocation("texCoord");
    normalAttr1 = diceProgram->attributeLocation("normal");
    matrixUniform1 = diceProgram->uniformLocation("matrix");
    matrixProjection1 = diceProgram->uniformLocation("proj");
    textureUniform1 = diceProgram->uniformLocation("tex");
    materialColorUniform1 = diceProgram->uniformLocation("matcol");


    /**
     *	TABLE SHADER PROGRAM
     */
    QGLShader *fshader2 = new QGLShader(QGLShader::Fragment, context(), this);
    const char *fsrc2 =
	    "varying highp vec4 frag_normal;\n"
	    "varying highp vec4 frag_pos;\n"
	    "varying highp vec4 texc;\n"
	    "uniform sampler2D tex;\n"
	    "void main(void)\n"
	    "{\n"
	    "    gl_FragColor = (texture2D(tex, texc.st).rgba);\n"
	    "}\n";
    fshader2->compileSourceCode(fsrc2);


    // table shader shares dice's vertexshader
    tableProgram->addShader(vshader1);
    tableProgram->addShader(fshader2);
    tableProgram->link();
    vertexAttr2 = tableProgram->attributeLocation("vertex");
    texCoordAttr2 = tableProgram->attributeLocation("texCoord");
    normalAttr2 = tableProgram->attributeLocation("normal");
    matrixUniform2 = tableProgram->uniformLocation("matrix");
    matrixProjection2 = tableProgram->uniformLocation("proj");
    textureUniform2 = tableProgram->uniformLocation("tex");

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
}


void PhysicsWidget::mousePressEvent(QMouseEvent* event)
{
    if (m_infoButtonRect.contains(event->pos())) {
        m_infoViewShown = !m_infoViewShown;
        return;
    }


#ifdef Q_OS_SYMBIAN
    if (m_closeButtonRect.contains(event->pos())) {

        emit exitPressed();
        return;
    }
#endif

    m_mousePressed = true;
    m_mousePos = QVector3D(((float)event->pos().x() / (float)width() - 0.5f) * 2.0f,
                           ((float)event->pos().y() / (float)height() - 0.5f) * -2.0f,
                           0.8f);
}


void PhysicsWidget::mouseReleaseEvent(QMouseEvent* event)
{
    Q_UNUSED(event);
    m_mousePressed = false;
}


void PhysicsWidget::mouseMoveEvent(QMouseEvent* event)
{
    m_mousePos = QVector3D(((float)event->pos().x() / (float)width()-0.5f) * 2.0f,
                           ((float)event->pos().y() / (float)height()-0.5f) * -2.0f,
                           0.8f);
}


void PhysicsWidget::createPlane(btVector3 planeVector, btScalar planeConstant)
{
    btStaticPlaneShape *planeShape = new btStaticPlaneShape(planeVector, planeConstant);
    btDefaultMotionState* planeMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));

    btRigidBody::btRigidBodyConstructionInfo planeRigidBodyCI(0, planeMotionState, planeShape, btVector3(0, 0, 0));

    btRigidBody *planeRigidBody = new btRigidBody(planeRigidBodyCI);
    m_DynamicsWorld->addRigidBody(planeRigidBody);

    m_PlaneShapes.push_back(planeShape);
    m_PlaneRigidBodies.push_back(planeRigidBody);
}


void PhysicsWidget::createWorld(int cubes)
{
    // Initialize the Bullet physics engine
    m_Broadphase = new btDbvtBroadphase();

    m_CollisionConfiguration = new btDefaultCollisionConfiguration();
    m_Dispatcher = new btCollisionDispatcher(m_CollisionConfiguration);
    m_Solver = new btSequentialImpulseConstraintSolver;

    // Create world and apply the gravity
    m_DynamicsWorld = new btDiscreteDynamicsWorld(m_Dispatcher, m_Broadphase, m_Solver, m_CollisionConfiguration);
    m_DynamicsWorld->setGravity(btVector3(0, 0, -9.81f));

    // Create ground shapes
    float boxWidthR = 1.0f * 0.92f;
    float boxHeightR = 1.0f / m_tableAspectRatio * 0.87f;

    createPlane(btVector3(0, 0, 1), 0);
    createPlane(btVector3(0, 0, -1), -3);
    createPlane(btVector3(1, 0, 0), -boxWidthR);
    createPlane(btVector3(-1, 0, 0), -boxWidthR);
    createPlane(btVector3(0, 1, 0), -boxHeightR);
    createPlane(btVector3(0, -1, 0), -boxHeightR);

    // Create the dice
    for (int i=0; i<cubes; i++) {
        addCube();
    }
}


void PhysicsWidget::deleteWorld()
{
    while(m_DiceRigidBodies.empty() == false) {
        removeCube();
    }

    foreach (btRigidBody *body, m_PlaneRigidBodies) {
        m_DynamicsWorld->removeRigidBody(body);
        delete body->getMotionState();
        delete body;
    }
    m_PlaneRigidBodies.clear();


    foreach (btCollisionShape *shape, m_PlaneShapes) {
        delete shape;
    }
    m_PlaneShapes.clear();


    delete m_DynamicsWorld;
    delete m_Solver;
    delete m_CollisionConfiguration;
    delete m_Dispatcher;
    delete m_Broadphase;
}


void PhysicsWidget::newCubeCount(int count)
{
    if(count == m_DiceRigidBodies.count()) {
	return;
    }
    else if(count > m_DiceRigidBodies.count()) {
        while(m_DiceRigidBodies.count() < count) {
	    addCube();
        }
    }
    else {
        while(m_DiceRigidBodies.count() > count) {
	    removeCube();
        }
    }
}


void PhysicsWidget::addCube()
{
    float box_size = COLLISION_BOX_SIZE * DICE_SCALE / 2;

    // random position for the new cube
    float x, y, z;
    x = ((float)(rand()&255) / 255.0f - 0.5f) * 1.0f;
    y = ((float)(rand()&255) / 255.0f - 0.5f) * 1.0f;
    z = 0.75f;

    btBoxShape *diceShape = new btBoxShape(btVector3(box_size, box_size, box_size));

    btDefaultMotionState *motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(x, y, z)));

    btScalar mass = INITIAL_MASS;

    btVector3 diceInertia(0, 0, 0);
    diceShape->calculateLocalInertia(mass, diceInertia);

    btRigidBody::btRigidBodyConstructionInfo diceRigidBodyCI(INITIAL_MASS, motionState, diceShape, diceInertia);
    diceRigidBodyCI.m_friction = INITIAL_SURFACE_FRICTION;
    diceRigidBodyCI.m_restitution = INITIAL_SURFACE_BOUNCYNESS;
    diceRigidBodyCI.m_linearSleepingThreshold = btScalar(0.0f);
    diceRigidBodyCI.m_angularSleepingThreshold = btScalar(0.0f);

    btRigidBody *diceRigidBody = new btRigidBody(diceRigidBodyCI);
    m_DynamicsWorld->addRigidBody(diceRigidBody);

    m_DiceShapes.push_back(diceShape);
    m_DiceRigidBodies.push_back(diceRigidBody);
}


void PhysicsWidget::removeCube()
{
    if (m_DiceRigidBodies.isEmpty()) {
        return;
    }

    btRigidBody *body = m_DiceRigidBodies.last();
    m_DynamicsWorld->removeRigidBody(body);
    delete body->getMotionState();
    delete body;
    m_DiceRigidBodies.pop_back();

    btCollisionShape *shape = m_DiceShapes.last();
    delete shape;
    m_DiceShapes.pop_back();
}


void PhysicsWidget::setSurfaceFriction(float friction)
{
    foreach (btRigidBody *body, m_DiceRigidBodies) {
        body->setFriction(friction);
    }
}


void PhysicsWidget::setSurfaceBouncyness(float bouncyness)
{
    foreach (btRigidBody *body, m_DiceRigidBodies) {
        body->setRestitution(bouncyness);
    }
}


void PhysicsWidget::step()
{
#if defined(Q_OS_SYMBIAN) || defined(Q_WS_HARMATTAN)
    QSensorReading *reading = m_Accelerometer->reading();
    qreal x = -reading->value(0).value<qreal>() * 5.0f;
    qreal y = reading->value(1).value<qreal>() * 5.0f;
    qreal z = -reading->value(2).value<qreal>() * 5.0f;

    // Restrict the gravity to 10g on each axis.
    if (x>10) x=10;
    else if(x<-10) x=-10;

    if (y>10) y=10;
    else if(y<-10) y=-10;

    if (z>10) z=10;
    else if(z<-10) z=-10;

    m_DynamicsWorld->setGravity(btVector3(y, x, z));
#endif

    QTime current_time = QTime::currentTime();
    int relTime = m_lastTime.msecsTo(current_time);
    m_lastTime = current_time;

    if (m_mousePressed) {
        foreach (btRigidBody *body, m_DiceRigidBodies) {
            btTransform diceTransform;
            body->getMotionState()->getWorldTransform(diceTransform);
            btVector3 pos = diceTransform.getOrigin();
            QVector3D delta = QVector3D(m_mousePos.x() - pos.x(), m_mousePos.y() - pos.y(), m_mousePos.z() - pos.z());
            delta.normalize();

            float sqr_distance = delta.x() * delta.x() + delta.y() * delta.y();
            delta *= relTime / (sqr_distance + 1.0f) * 2;

            body->applyForce(btVector3(delta.x(), delta.y(), delta.z()), btVector3(0,0,0));
        }
    }

    m_DynamicsWorld->stepSimulation(relTime * 0.001);
}


void PhysicsWidget::RenderDice(btTransform *transform)
{
    float ftemp[16];
    transform->getOpenGLMatrix(ftemp);

    QMatrix4x4 modelview(ftemp[0], ftemp[1], ftemp[2], ftemp[3],
                         ftemp[4], ftemp[5], ftemp[6], ftemp[7],
                         ftemp[8], ftemp[9], ftemp[10], ftemp[11],
                         ftemp[12], ftemp[13], ftemp[14]-m_viewDistance, ftemp[15]);

    modelview = modelview.transposed();
    diceProgram->setUniformValue(matrixUniform1, modelview);

    // use white as materialcolor for first mesh (the body of a dice)
    QVector3D col( 1.0f, 1.0f, 1.0f );
    diceProgram->setUniformValue(materialColorUniform1, col);

    ExampleLoader::Mesh *l = m_diceMesh;
    while (l) {
        diceProgram->setAttributeArray( vertexAttr1, l->getVertices()->coord, 3,sizeof( ExampleLoader::Vertex ));
        diceProgram->setAttributeArray( normalAttr1, l->getVertices()->normal, 3,sizeof( ExampleLoader::Vertex ));
        diceProgram->setAttributeArray( texCoordAttr1, l->getVertices()->uv, 2, sizeof( ExampleLoader::Vertex));
        glDrawElements(GL_TRIANGLES, l->getIndexCount(), GL_UNSIGNED_SHORT, l->getIndices());
        l = l->nextMesh;

        // use darkgray as materialcolor for second mesh (eyes)
        col = QVector3D(0.3f, 0.3f, 0.3f);
        diceProgram->setUniformValue(materialColorUniform1, col);
    }
}


void PhysicsWidget::RenderDiceShadow(btTransform *transform)
{
    float ftemp[16];
    transform->getOpenGLMatrix(ftemp);

    QMatrix4x4 modelview(ftemp[0], ftemp[1], ftemp[2], ftemp[3],
                         ftemp[4], ftemp[5], ftemp[6], ftemp[7],
                         ftemp[8], ftemp[9], ftemp[10], ftemp[11],
                         ftemp[12], ftemp[13], ftemp[14]-m_viewDistance, ftemp[15]);

    modelview = modelview.transposed();
    shadowProgram->setUniformValue(shadowMatrixUniform, modelview);
    shadowProgram->setAttributeArray(shadowVertexAttr, cube_vertices, 3, 0);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, cube_indices);
}


void PhysicsWidget::paintGL()
{
    // update the physisc
    step();

    QPainter painter;
    painter.begin(this);
    painter.beginNativePainting();

    glViewport(0,0, width(), height());

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );


    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glDisable(GL_ALPHA);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);


    float w = (float)this->width();
    float h = (float)this->height();

    // draw only the area where table is visible.
    int useHeight = (int)( (float)width() / m_tableAspectRatio );

    if (useHeight<height()) {
	glEnable( GL_SCISSOR_TEST );
        glScissor(0, (height()-useHeight) / 2, width(), useHeight);
    }
    else {
        glDisable(GL_SCISSOR_TEST);
    }

    QMatrix4x4 projection;
    projection.ortho(-1.0f, 1.0f, -h/w, h/w, 0.1f, 20.0f);


    // RENDER THE TABLE
    tableProgram->bind();
    tableProgram->enableAttributeArray(normalAttr2);
    tableProgram->enableAttributeArray(vertexAttr2);
    tableProgram->enableAttributeArray(texCoordAttr2);
    tableProgram->setUniformValue( matrixProjection2, projection );

    QMatrix4x4 modelview;
    modelview.setToIdentity();
    modelview.scale( 1.0f, 1.0f / m_tableAspectRatio, 1.0f );
    modelview.translate(0.0f, 0.0f, -m_viewDistance);
    tableProgram->setUniformValue(matrixUniform2, modelview);

    // draw the ground
    glBindTexture(GL_TEXTURE_2D, m_tableTexture);
    tableProgram->setUniformValue(textureUniform2, 0);    // use texture unit 0
    tableProgram->setAttributeArray( vertexAttr2, table_vert, 3,0);
    tableProgram->setAttributeArray( normalAttr2, table_normal, 3,0);
    tableProgram->setAttributeArray( texCoordAttr2, table_uv, 2, 0);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    tableProgram->disableAttributeArray(normalAttr2);
    tableProgram->disableAttributeArray(vertexAttr2);
    tableProgram->disableAttributeArray(texCoordAttr2);
    tableProgram->release();



    // RENDER THE SHADOWS
    shadowProgram->bind();
    shadowProgram->enableAttributeArray(shadowVertexAttr);
    shadowProgram->setUniformValue( shadowMatrixProjection, projection );

    glDisable( GL_DEPTH_TEST );				      // dont use depthbuffer for the shadows
    glBindTexture(GL_TEXTURE_2D, m_tableTexture);
    shadowProgram->setUniformValue(shadowTextureUniform, 0);  // use texture unit 0

    foreach (btRigidBody *body, m_DiceRigidBodies) {
        btTransform diceTransform;
        body->getMotionState()->getWorldTransform(diceTransform);
        RenderDiceShadow(&diceTransform);
    }

    shadowProgram->disableAttributeArray(shadowVertexAttr);
    shadowProgram->release();
    glEnable( GL_DEPTH_TEST );

    // RENDER THE DICES
    diceProgram->bind();

    diceProgram->enableAttributeArray(normalAttr1);
    diceProgram->enableAttributeArray(vertexAttr1);
    diceProgram->enableAttributeArray(texCoordAttr1);
    diceProgram->setUniformValue( matrixProjection1, projection );

    // Draw the dices.
    glBindTexture(GL_TEXTURE_2D, m_combinedTexture);
    diceProgram->setUniformValue(textureUniform1, 0);   // use texture unit 0


    foreach (btRigidBody *body, m_DiceRigidBodies) {
        btTransform diceTransform;
        body->getMotionState()->getWorldTransform(diceTransform);
        RenderDice(&diceTransform);
    }

    diceProgram->disableAttributeArray(normalAttr1);
    diceProgram->disableAttributeArray(vertexAttr1);
    diceProgram->disableAttributeArray(texCoordAttr1);
    diceProgram->release();

    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);

    painter.endNativePainting();

    // Paint the info and exit buttons.
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    int tableHeight = (height() - useHeight) / 2;
    m_infoButtonRect = QRect(0.005f * width(),
                             tableHeight + useHeight * 0.005f,
                             width() * 0.05f,
                             width() * 0.05f);

    painter.drawPixmap(m_infoButtonRect, *m_infoButtonPixmap);

    if (m_infoViewShown) {
        painter.setPen(Qt::transparent);

        QRect infoRect = QRect(0.05f * width(),
                               tableHeight + useHeight * 0.1f,
                               width() * 0.9f,
                               useHeight * 0.8f);

        QRadialGradient radialGradient(width() / 2,
                                       height() / 2,
                                       width() * 0.5f,
                                       width() / 2 * 0.7f,
                                       height() / 2);
        radialGradient.setColorAt(0.0, QColor(10, 10, 10, 180));
        radialGradient.setColorAt(1.0, Qt::transparent);
        painter.setBrush(radialGradient);
        painter.drawRect(infoRect);

        QFont font;
        font.setPixelSize(useHeight * 0.028f);
        painter.setFont(font);
        painter.setPen(Qt::yellow);

        qreal textWidth = width() * 0.8f;
        if (!qFuzzyCompare(m_infoText.textWidth(), textWidth)) {
            m_infoText.setTextWidth(textWidth);
        }

        painter.drawStaticText(infoRect.x() * 1.5f,
                               infoRect.y() + useHeight * 0.07f,
                               m_infoText);
    }

#ifdef Q_OS_SYMBIAN
    // Paint the exit button
    m_closeButtonRect = QRect(width() - 0.05f * width(),
                              tableHeight + useHeight * 0.01f,
                              width() * 0.04f,
                              width() * 0.04f);

    painter.drawPixmap(m_closeButtonRect, *m_closeButtonPixmap);
#endif

    painter.end();

    glFinish();
    swapBuffers();
}
