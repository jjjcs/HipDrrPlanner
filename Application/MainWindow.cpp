#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QPushButton>
#include <QDebug>
#include <QDir>
#include <QTime>
#include <QDebug>




MainWindow::MainWindow() :m_Ui(new Ui::MainWindow)
{
    m_Ui->setupUi(this);
    connect(m_Ui->pButtonLogNormalization,
        &QPushButton::clicked, this, &MainWindow::SlotLogNormalization);
    connect(m_Ui->pButtonTruncationCorrection, 
        &QPushButton::clicked, this, &MainWindow::SlotTruncationCorrection);
    connect(m_Ui->pButtonFDKReconstruction, 
        &QPushButton::clicked, this, &MainWindow::SlotFDKReconstruction);
    connect(m_Ui->pButtonTruncationCorrection_cuda,
        &QPushButton::clicked, this, &MainWindow::SlotTruncationCorrection_cuda);
    connect(m_Ui->pButtonParkerWeighting,
        &QPushButton::clicked, this, &MainWindow::SlotParkerWeighting);
    connect(m_Ui->pButtonParkerWeighting_cuda,
        &QPushButton::clicked, this, &MainWindow::SlotParkerWeighting_cuda);
}

MainWindow::~MainWindow()
{
    delete m_Ui;
}


void MainWindow::SlotLogNormalization()
{
    LogNormalization();
}

void MainWindow::LogNormalization()
{

}


void MainWindow::SlotTruncationCorrection()
{
    TruncationCorrection();
}

void MainWindow::TruncationCorrection()
{

}

void MainWindow::SlotTruncationCorrection_cuda()
{
    TruncationCorrection_cuda();
}

void MainWindow::TruncationCorrection_cuda()
{

}

void MainWindow::SlotFDKReconstruction()
{
    FDKReconstruction();
}

void MainWindow::FDKReconstruction()
{

}

void MainWindow::SlotParkerWeighting()
{
    ParkerWeighting();
}

void MainWindow::ParkerWeighting()
{

}

void MainWindow::SlotParkerWeighting_cuda()
{
    ParkerWeighting_cuda();
}

void MainWindow::ParkerWeighting_cuda()
{

}