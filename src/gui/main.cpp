#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>

#include "weather_node.h"

int main(int argc, char* argv[]) {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;
    qmlRegisterType<QRosNode>("backend", 1, 0, "Backend");
    const QUrl url(QStringLiteral("main.qml"));
    QObject::connect(
        &engine, &QQmlApplicationEngine::objectCreated, &app,
        [url](QObject* obj, const QUrl& objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        },
        Qt::QueuedConnection);
    engine.load(url);

    // QGuiApplication app(argc, argv);
    // QQuickView view;
    // view.setSource(QUrl(":/qml/mail.qml"));
    // if (view.errors().isEmpty() == false) {
    //     return -1;
    // }

    // QRosNode node(nullptr, argc, argv);
    // view.rootContext()->setContextProperty("backend", &node);
    // view.show();

    return app.exec();
}
