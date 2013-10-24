/**
 * Copyright (c) 2011 Nokia Corporation and/or its subsidiary(-ies).
 * All rights reserved.
 *
 * OpenGL ES 2.0 and physics example.
 *
 * For the applicable distribution terms see the license.txt -file, included in
 * the distribution.
 */

#ifndef PHYSICSWIDGET_H
#define PHYSICSWIDGET_H

#include <QGLWidget>
#include <QVector3D>
#include <QTime>
#include <QStaticText>
#include <btBulletDynamicsCommon.h>
#include "Mesh.h"

#if defined(Q_OS_SYMBIAN) || defined(Q_WS_HARMATTAN)
#include <QAccelerometer>

QTM_USE_NAMESPACE
#endif

class QTimer;
class QGLShaderProgram;

// Scaling factor for the dices
#define DICE_SCALE			1.5f

// How many cubes the world has in the beginning
#define INITIAL_CUBE_COUNT		5

// Friction of the collisions (between all objects)
#define INITIAL_SURFACE_FRICTION	3.0f

// Bouncyness of the collisions (between all objects)
#define INITIAL_SURFACE_BOUNCYNESS	0.1f

// Mass of the dice
#define INITIAL_MASS                    0.8f

// The "r" of the box used in collision calculations.
#define COLLISION_BOX_SIZE              0.13f


class PhysicsWidget : public QGLWidget
{
    Q_OBJECT

public:
    PhysicsWidget(QWidget *parent = 0);
    virtual ~PhysicsWidget();

    void addCube();
    void removeCube();
    void createPlane(btVector3 planeVector, btScalar planeConstant);

    void createWorld(int cubes);
    void deleteWorld();

    void RenderDice(btTransform *transform);
    void RenderDiceShadow(btTransform *transform);

public slots:
    void newCubeCount(int count);
    void setSurfaceFriction(float friction);
    void setSurfaceBouncyness(float bouncyness);

signals:
    void exitPressed();

protected:
    void step();
    void paintGL();
    void initializeGL();
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent *event );
    void mouseMoveEvent(QMouseEvent* event);

private:

#if defined(Q_OS_SYMBIAN) || defined(Q_WS_HARMATTAN)
    QAccelerometer *m_Accelerometer;
#endif

    // Bullet engine objects
    btBroadphaseInterface* m_Broadphase;
    btDefaultCollisionConfiguration* m_CollisionConfiguration;
    btCollisionDispatcher* m_Dispatcher;

    btSequentialImpulseConstraintSolver* m_Solver;
    btDiscreteDynamicsWorld* m_DynamicsWorld;

    QList<btCollisionShape*> m_PlaneShapes;
    QList<btRigidBody*> m_PlaneRigidBodies;

    QList<btCollisionShape*> m_DiceShapes;
    QList<btRigidBody*> m_DiceRigidBodies;

    QVector3D m_mousePos;
    bool m_mousePressed;

    QTime m_lastTime;

    float m_tableAspectRatio;	     // aspect ratio for table
    float m_viewDistance;	     // how far the camera is from the table.
    ExampleLoader::Mesh *m_diceMesh; // Dice Mesh container

    // Shader program for rendering the dices
    QGLShaderProgram *diceProgram;
    int vertexAttr1;
    int texCoordAttr1;
    int normalAttr1;
    int matrixUniform1;
    int textureUniform1;
    int matrixProjection1;
    int materialColorUniform1;

    // Shader program for rendering the table
    QGLShaderProgram *tableProgram;
    int vertexAttr2;
    int texCoordAttr2;
    int normalAttr2;
    int matrixUniform2;
    int textureUniform2;
    int matrixProjection2;

    // Shader program for rendering the shadows
    QGLShaderProgram *shadowProgram;
    int shadowVertexAttr;
    int shadowTextureUniform;
    int shadowMatrixUniform;
    int shadowMatrixProjection;

    GLuint m_combinedTexture;
    GLuint m_tableTexture;

    bool m_infoViewShown;

    QRect m_infoButtonRect;
    QPixmap *m_infoButtonPixmap;
    QStaticText m_infoText;

#ifdef Q_OS_SYMBIAN
    QRect m_closeButtonRect;
    QPixmap *m_closeButtonPixmap;
#endif

    QTimer* timer;
};

#endif // PHYSICSWIDGET_H
