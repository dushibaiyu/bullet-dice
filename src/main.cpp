/**
 * Copyright (c) 2011 Nokia Corporation and/or its subsidiary(-ies).
 * All rights reserved.
 *
 * OpenGL ES 2.0 and physics example.
 *
 * For the applicable distribution terms see the license.txt -file, included in
 * the distribution.
 */

#include <QApplication>
#include "PhysicsWidget.h"

// Lock orientation in Symbian
#ifdef Q_OS_SYMBIAN
    #include <eikenv.h>
    #include <eikappui.h>
    #include <aknenv.h>
    #include <aknappui.h>
#endif


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // Lock orientation in Symbian
#ifdef Q_OS_SYMBIAN
    CAknAppUi* appUi = dynamic_cast<CAknAppUi*> (CEikonEnv::Static()->AppUi());
    TRAP_IGNORE(if (appUi) {
        appUi->SetOrientationL(CAknAppUi::EAppUiOrientationLandscape);
    });
#endif

    PhysicsWidget widget;

    QObject::connect(&widget, SIGNAL(exitPressed()), &app, SLOT(quit()));

#if defined(Q_WS_HARMATTAN) || defined(Q_OS_SYMBIAN)
    widget.showFullScreen();
#else
    widget.showMaximized();
#endif

    return app.exec();
}
