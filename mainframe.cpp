#define MAX_LINES 156

#include "mainframe.h"
#include "ui_mainframe.h"
#include "imageform.h"

#include <QFileDialog>
#include <QPainter>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "calibrationZhang.h"
#include "rectification.h"


using namespace std;

MainFrame::MainFrame(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MainFrame)
{
    ui->setupUi(this);

    _q_pFormFocused     = 0;

    //객체 맴버의 초기화


    //리스트 출력창을 안보이게    
    ui->listWidget->setVisible(false);
    this->adjustSize();

    //UI 활성화 갱신
    UpdateUI();
}

MainFrame::~MainFrame()
{ 
    delete ui;         

    for(auto& item : _lImageForm)
        delete item;

}

void MainFrame::CloseImageForm(ImageForm *pForm)
{
    //ImageForm 포인터 삭제
    unsigned int idx = std::find(_lImageForm.begin(), _lImageForm.end(), pForm) - _lImageForm.begin();
    if(idx != _lImageForm.size())
    {
        delete _lImageForm[idx];
        _lImageForm.erase(_lImageForm.begin() + idx);
    }

    //활성화 ImageForm 초기화
    _q_pFormFocused     = 0;

    //관련 객체 삭제

    //UI 활성화 갱신
    UpdateUI();
}

void MainFrame::UpdateUI()
{    
    if(ui->tabWidget->currentIndex() == 0)
    {
    }
    else if(ui->tabWidget->currentIndex() == 1)
    {  
    }
    else if(ui->tabWidget->currentIndex() == 2)
    {        
    }
    else if(ui->tabWidget->currentIndex() == 3)
    {
    }
}

void MainFrame::OnMousePos(const int &nX, const int &nY, ImageForm* q_pForm)
{
}

void MainFrame::focusInEvent(QFocusEvent * event)
{
    Q_UNUSED(event) ;

    UpdateUI();
}

void MainFrame::closeEvent(QCloseEvent* event)
{
    //생성된 ImageForm을 닫는다.
    for(int i=0; i< _lImageForm.size(); i++)
        delete _lImageForm[i];

    //리스트에서 삭제한다.
    _lImageForm.clear();
}


void MainFrame::on_buttonOpen_clicked()
{
    //이미지 파일 선택
    QFileDialog::Options    q_Options   =  QFileDialog::DontResolveSymlinks  | QFileDialog::DontUseNativeDialog; // | QFileDialog::ShowDirsOnly
    QString                 q_stFile    =  QFileDialog::getOpenFileName(this, tr("Select a Image File"),  "./data", "Image Files(*.bmp *.ppm *.pgm *.png)",0, q_Options);

    if(q_stFile.length() == 0)
        return;

    //이미지 출력을 위한 ImageForm 생성    
    ImageForm*              q_pForm   = new ImageForm(q_stFile, "OPEN", this);

    _lImageForm.push_back(q_pForm);
    q_pForm->show();

    //UI 활성화 갱신
    UpdateUI();
}

void MainFrame::on_buttonDeleteContents_clicked()
{
    //생성된 ImageForm을 닫는다.
    for(int i=_lImageForm.size()-1; i>=0; i--)
        delete _lImageForm[i];

    //리스트에서 삭제한다.
    _lImageForm.clear();

    ui->listWidget->clear();
}

void MainFrame::on_tabWidget_currentChanged(int index)
{
    static int nOld = -1;

    if(nOld == 0)
    {

    }
    else if(nOld == 1)
    {

    }
    nOld = index;

    //UI 활성화 갱신
    UpdateUI();
}

void MainFrame::on_buttonShowList_clicked()
{
    static int nWidthOld = ui->tabWidget->width();

    if(ui->listWidget->isVisible())
    {
        nWidthOld = ui->listWidget->width();
        ui->listWidget->hide();
        this->adjustSize();
    }
    else
    {        
        ui->listWidget->show();
        QRect q_rcWin = this->geometry();

        this->setGeometry(q_rcWin.left(), q_rcWin.top(), q_rcWin.width()+nWidthOld, q_rcWin.height());
    }
}

void MainFrame::on_pushCalibration_clicked()
{
    //입력 특징점 파일 선택
    QFileDialog::Options  q_Options =  QFileDialog::DontResolveSymlinks  |
                                       QFileDialog::DontUseNativeDialog;
    QStringList stlFile  =  QFileDialog::getOpenFileNames(this, tr("Select Feature Files"),
                                                        "./data", "Text Files(*.txt)",0, q_Options);
    if(stlFile.length() == 0)
        return;

    vector<Eigen::MatrixXf> lF;

    for(int i=0; i<stlFile.count(); i++)
    {
        //특징점 읽어들임
        FILE*   fp  = fopen(stlFile.value(i).toStdString().c_str(),"rt");
        int     row = 0;

        std::vector<Eigen::Vector2f> points;
        Eigen::Vector2f vTmp;

        while(fscanf(fp, "%f %f\n", &vTmp(0), &vTmp(1)) == 2)
        {   
            // std::cout << vTmp(0) << '\n';
            // std::cout << vTmp(1) << "\n\n";
            points.push_back(vTmp);
        }
        fclose(fp);
        Eigen::MatrixXf mTmp(points.size(), 2);
        
        for (size_t r = 0; r < points.size(); r++)
        {
            mTmp.row(r) = points[r];
        }
        
        lF.push_back(mTmp);
    }

    /*
    QStringList stlFile2  =  QFileDialog::getOpenFileNames(this, tr("Select Feature Files"),
                                                        "./data", "Text Files(*.txt)",0, q_Options);
    if(stlFile2.length() == 0)
        return;

    vector<Eigen::MatrixXf> lF2;

    for(int i=0; i<stlFile2.count(); i++)
    {
        //특징점 읽어들임
        FILE*   fp  = fopen(stlFile2.value(i).toStdString().c_str(),"rt");
        int     row = 0;

        std::vector<Eigen::Vector2f> points;
        Eigen::Vector2f vTmp;

        while(fscanf(fp, "%f %f\n", &vTmp(0), &vTmp(1)) == 2)
        {   
            // std::cout << vTmp(0) << '\n';
            // std::cout << vTmp(1) << "\n\n";
            points.push_back(vTmp);
        }
        fclose(fp);
        Eigen::MatrixXf mTmp(points.size(), 2);
        
        for (size_t r = 0; r < points.size(); r++)
        {
            mTmp.row(r) = points[r];
        }
        
        lF2.push_back(mTmp);
    }
    */ // Rectification

    //입력 모델점 파일 선택
    KString stPath   =  KString(stlFile.value(0).toStdString().c_str()).FilePathOnly();
    QString stModel  =  QFileDialog::getOpenFileName(this, tr("Select a Model File"),
                                                   stPath.Address(), "Text Files(*.txt)",0, q_Options);
    if(stModel.length() == 0)
        return;

    //특징점 읽어들임
    std::vector<Eigen::Vector2f> modelPoints;
    FILE* fp = fopen(stModel.toStdString().c_str(), "rt");
    if(!fp)
        return;

    Eigen::Vector2f vTmp;
    while(fscanf(fp, "%f %f", &vTmp(0), &vTmp(1)) == 2)
    {
        // std::cout << vTmp(0) << '\n';
        // std::cout << vTmp(1) << "\n\n";
        modelPoints.push_back(vTmp);
    }
    fclose(fp);

    Eigen::MatrixXf mM(modelPoints.size(), 2);
    for (size_t r = 0; r < modelPoints.size(); r++)
    {
        mM.row(r) = modelPoints[r];
    }

    //진행 표시
    if(ui->listWidget->isVisible() == false)
        on_buttonShowList_clicked();
    ui->listWidget->addItem("");
    ui->listWidget->addItem(KString::Format(">> Finish Loading : %d images, %d features",
                                            stlFile.count(), mM.cols()).Address());
    ui->listWidget->show();
    ui->listWidget->addItem(QString(">> Start Calibration..."));
    ui->listWidget->show();
    
    // AX=ZB

    //1. Calibration
    Eigen::Matrix3d Param;
    std::vector<double> Distortion;
    KCalibrationZhang calib;
    std::vector<Eigen::Matrix4f> Extrinsic;
    
    calib.doCalib(lF, mM);
    calib.getParam(Param, Distortion);
    calib.getExtrinsic(Extrinsic);
    
    ui->listWidget->addItem(QString(">> End Calibration"));
    ui->listWidget->addItem(QString(">> Start Hand-eye calibration ..."));

    // Apply AX=ZB
    

    /* Rectification Example
    Eigen::Matrix3d leftParam, rightParam;
    std::vector<double> leftDistortion, rightDistortion;

    //아래를 완성하면 됩니다.
    KCalibrationZhang calib;

    std::vector<Eigen::Matrix4f> lExtrinsic;
    std::vector<Eigen::Matrix4f> rExtrinsic;

    calib.doCalib(lF, mM);
    std::cout << "Left finished!\n";
    calib.getParam(leftParam, leftDistortion);
    std::cout << "Param loaded\n";
    calib.getExtrinsic(lExtrinsic);
    std::cout << "Extrinsic loaded\n";

    KCalibrationZhang calibR;

    calibR.doCalib(lF2, mM);
    calibR.getParam(rightParam, rightDistortion);
    calibR.getExtrinsic(rExtrinsic);
    std::cout << "Right finished!\n";

    ui->listWidget->addItem(QString(">> End Calibration"));
    ui->listWidget->addItem(QString(">> Start Ratification ..."));

    Rectification rectifier;

    rectifier.setParam(leftParam,rightParam,leftDistortion,rightDistortion,lExtrinsic,rExtrinsic,mM);
    rectifier.run();
    ui->listWidget->addItem(QString(">> End rectification"));
    

    // Codes for calib evaluation
    calib.evalParamDiff();
    calib.evalCoordDiff();
    std::vector<float> evals;
    evals = calib.getEval();
    
    ui->listWidget->addItem(QString(">> ===== Left Param =====")
        .arg(evals[0])  // Param Error Mean
        .arg(evals[1])); // Param Error Std
    ui->listWidget->addItem(QString(">> ===== Projection Error(pixel) =====\n Mean: %1 \n RMSE: %2")
    .arg(evals[2])  // Projection Error Mean
    .arg(evals[3])); // Projection Error Std

    calibR.evalParamDiff();
    calibR.evalCoordDiff();
    std::vector<float> evalsR;
    evalsR = calibR.getEval();
    
    ui->listWidget->addItem(QString(">> ===== Right Param =====")
        .arg(evalsR[0])  // Param Error Mean
        .arg(evalsR[1])); // Param Error Std
    ui->listWidget->addItem(QString(">> ===== Projection Error(pixel) =====\n Mean: %1 \n RMSE: %2")
    .arg(evalsR[2])  // Projection Error Mean
    .arg(evalsR[3])); // Projection Error Std

    ui->listWidget->addItem(QString(">> Start rectification visualize ..."));
    QStringList bmpFiles = QFileDialog::getOpenFileNames(this, tr("Select Left BMP Images"),
    "./data", "BMP Files(*.bmp)", 0, q_Options);
    if (bmpFiles.length() == 0) return;

    std::vector<cv::Mat> leftImages;
    for (const QString& file : bmpFiles)
    {
        cv::Mat img = cv::imread(file.toStdString(), cv::IMREAD_COLOR);
        if (!img.empty())
        {
            leftImages.push_back(img);
        }
    }

    QStringList RbmpFiles = QFileDialog::getOpenFileNames(this, tr("Select Right BMP Images"),
    "./data", "BMP Files(*.bmp)", 0, q_Options);
    if (RbmpFiles.length() == 0) return;

    std::vector<cv::Mat> rightImages;
    for (const QString& file : RbmpFiles)
    {
        cv::Mat img = cv::imread(file.toStdString(), cv::IMREAD_COLOR);
        if (!img.empty())
        {
            rightImages.push_back(img);
        }
    }

    rectifier.visualize(leftImages, rightImages);   
    */    //Rectification example
}

