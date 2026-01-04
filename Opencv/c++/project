#include <windows.h>	// ballsound 함수를 위한 헤더
#include "opencv2/opencv.hpp"
#include <iostream>
#include <ctime>        // 시간 관련 헤더
#include <vector>
#include <cmath>

#define A4 440 // 라
#define B4 494 // 시
#define C5 523 // 도
#define D5 587 // 레
#define E5 659 // 미
#define F5 698 // 파
#define G5 784 // 솔
#define A5 880 // 라


// 축이동
float posi_data[] = { 0.2,-0.2 };
float dx, dy;


struct Ball {
    cv::Point position;
    cv::Mat photo;
    int radius;
    bool active;
    clock_t respawn;

    Ball() {
        this->position = cv::Point();
        this->active = false;
        this->respawn = clock();
    }
};
struct ColorBall :Ball
{
    int blue, green, red;
    clock_t color_t;

    ColorBall() {
        this->red = rand() % 256;
        this->green = rand() % 256;
        this->blue = rand() % 256;
        this->color_t = clock();
    }
};

struct Note {
    int frequency;
    int duration;
};

// 테트리스 브금
Note tetris[] = {
    
    {E5, 400}, {B4, 200}, {C5, 200}, {D5, 400}, {C5, 200}, {B4, 200},
    {A4, 400}, {A4, 200}, {C5, 200}, {E5, 400}, {D5, 200}, {C5, 200},
    {B4, 400}, {B4, 200}, {C5, 200}, {D5, 400}, {E5, 400},
    {C5, 400}, {A4, 400}, {A4, 400}

};

void playMusic() {
    while (true) { 
        for (auto& n : tetris) {
            Beep(n.frequency, n.duration);
            
            Sleep(20);
        }
    }
}


cv::Point getRandomPosition(int width, int height, int radius) {
    int x = rand() % (width - 2 * radius) + radius;
    int y = rand() % (height - 2 * radius) + radius;
    return cv::Point(x, y);
}

void ballsound(int s) {
    // 성공은 1000 실패는 200
    Beep(s, 60); // 음을 0.07초간 재생
    return;
}

void runProject() CV_NOEXCEPT {
    srand((unsigned int)time(NULL));

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "웹캠이 없습니다.";
        return;
    }

    int width = cvRound(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    //std::cout << "가로>>"<<width<<"\n"; //640
    int height = cvRound(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    //std::cout << "세로>>" << height << "\n"; //480
    cv::Mat prev_gray;

    // 포토볼 생성
    Ball Photoball;
    int flip_ran = 1;
    Photoball.radius = rand() % 20 + 15;
    Photoball.photo = cv::imread("picture.png");

    // 랜덤 컬러볼 생성
    ColorBall Ran_Color_Ball;
    Ran_Color_Ball.radius = rand() % 20 + 15;
    Ran_Color_Ball.photo = cv::imread("cat.bmp");

    // 놀라는 모핑
    cv::Mat surprise_img = cv::imread("surprise.png"); // 놀라는 사진 추가
    if (surprise_img.empty()) surprise_img = Ran_Color_Ball.photo; // 없으면 기본사진으로 대체


    // 이미지 로드 실패 확인
    if (Photoball.photo.empty()) {
        std::cerr << "이미지를 불러올 수 없습니다! 경로를 확인하세요.";
        return;
    }

    Photoball.position = getRandomPosition(width, height, Photoball.radius);
    Ran_Color_Ball.position = getRandomPosition(width, height, Ran_Color_Ball.radius);

    int score = 0;
    int color_score;
    while (true) {
        cv::Mat frame, gray_frame, diff, thresh;
        cap >> frame;
        if (frame.empty()) break;
        cv::flip(frame, frame, 1);
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray_frame, gray_frame, cv::Size(15, 15), 0);

        if (prev_gray.empty()) {
            gray_frame.copyTo(prev_gray);
            continue;
        }

        cv::absdiff(prev_gray, gray_frame, diff);
        cv::threshold(diff, thresh, 25, 255.0, cv::THRESH_BINARY);

        // --- 공 충돌 및 그리기 로직 ---
        int diameter_p = Photoball.radius * 2;
        int x1 = Photoball.position.x - Photoball.radius;
        int y1 = Photoball.position.y - Photoball.radius;

        int diameter_r = Ran_Color_Ball.radius * 2;
        int x2 = Ran_Color_Ball.position.x - Ran_Color_Ball.radius;
        int y2 = Ran_Color_Ball.position.y - Ran_Color_Ball.radius;

        // 화면 밖으로 나가지 않도록 보정된 ROI 영역 (Safety Check)
        cv::Rect ballRect_1(x1, y1, diameter_p, diameter_p);
        cv::Rect ballRect_2(x2, y2, diameter_r, diameter_r);

        /*cv::circle(frame, Ran_Color_Ball.position, Ran_Color_Ball.radius,
            cv::Scalar(Ran_Color_Ball.blue, Ran_Color_Ball.green, Ran_Color_Ball.red), -1);*/

            // 포토볼에 내부 쿨타임
        if (!Photoball.active) {
            clock_t current_time_p = clock();
            clock_t diff_timt_p = current_time_p - Photoball.respawn;
            double second_p = diff_timt_p / CLOCKS_PER_SEC; //ms >> second 
            if (second_p >= 0.01) {
                Photoball.active = true;
            }
        }
        // 랜덤볼에 내부 쿨타임
        if (!Ran_Color_Ball.active) {
            clock_t current_time_r = clock();
            clock_t diff_time_r = current_time_r - Ran_Color_Ball.respawn;
            double second_r = diff_time_r / CLOCKS_PER_SEC; //ms >> second 
            if (second_r >= 0.01) {
                Ran_Color_Ball.active = true;
            }
        }

        // Photoball사진 입히기 (마스킹)
        cv::Mat ball_img;
        // Photoball사진 크기 조절
        cv::resize(Photoball.photo, ball_img, cv::Size(diameter_p, diameter_p)); 

        cv::Mat mask = cv::Mat::zeros(ball_img.size(), CV_8UC1);
        cv::circle(mask, cv::Point(Photoball.radius, Photoball.radius), 
            Photoball.radius, cv::Scalar(255), -1, cv::LINE_AA);

        // Ran_Color_Ball 사진 준비 및 크기 조절
        cv::Mat ball_img_r;
        cv::resize(Ran_Color_Ball.photo, ball_img_r, cv::Size(diameter_r, diameter_r));

        
        // Ran_Color_Ball 내부에 저장된 RGB 값을 사용.
        cv::Scalar random_color(Ran_Color_Ball.blue, 
                                Ran_Color_Ball.green, 
                                Ran_Color_Ball.red
        );
        cv::Mat color_layer(ball_img_r.size(), ball_img_r.type(), random_color);

        cv::Mat blended_img;
        double alpha = 0.5; // 사진 비중
        cv::addWeighted(ball_img_r, alpha, color_layer, 1.0 - alpha, 0, blended_img);

        // 원형 마스크 생성
        cv::Mat mask_r = cv::Mat::zeros(blended_img.size(), CV_8UC1);
        cv::circle(mask_r,
            cv::Point(Ran_Color_Ball.radius, Ran_Color_Ball.radius),
            Ran_Color_Ball.radius,
            cv::Scalar(255), -1, cv::LINE_AA);

        // 최종 결과물을 프레임(배경)에 합성
        blended_img.copyTo(frame(ballRect_2), mask_r);

        

        // 축 이동

        if (Photoball.position.x >= 320) dx += posi_data[0];
        else dx += posi_data[1];
        if (Photoball.position.y >= 240) dy += posi_data[0];
        else dy += posi_data[1];
        /*float data[] = { 1, 0, (float)dx, 0, 1,(float)dy };*/

        cv::Mat img_normal, img_surprise;
        cv::resize(Photoball.photo, img_normal, cv::Size(diameter_p, diameter_p));
        cv::resize(surprise_img, img_surprise, cv::Size(diameter_p, diameter_p));

        double max_dist = Photoball.radius * 0.7;
        double dist_ratio = abs(dx) / max_dist;
        if (dist_ratio > 1.0) dist_ratio = 1.0;

        // alpha가 1.0이면 기본, 0.0이면 놀람
        double morph_alpha = 1.0;
        if (dist_ratio > 0.5) {
            // 절반(0.5)을 넘는 순간부터 1.0에서 0.0으로 줄어듦
            morph_alpha = 1.0 - (dist_ratio - 0.5) * 2.0;
        }

        // 모핑
        cv::Mat morphed_ball;
        cv::addWeighted(img_normal, morph_alpha, img_surprise, 
            1.0 - morph_alpha, 0, morphed_ball);

        // 축이동(Warping) 적용
        float data[] = { 1, 0, (float)dx, 0, 1, (float)dy };
        cv::Mat M(2, 3, CV_32F, data);
        cv::Mat translated_ball;
        cv::warpAffine(morphed_ball, translated_ball, M, morphed_ball.size());

        //  마스킹하여 화면에 출력
        translated_ball.copyTo(frame(ballRect_1), mask);

       

        // ROI가 화면 내부에 있을 때만 처리
        if ((ballRect_1 & cv::Rect(0, 0, width, height)) == ballRect_1) {
            // 충돌 감지 (움직임 확인)
            cv::Mat roiThresh_1 = thresh(ballRect_1);
            int movementPixels_1 = cv::countNonZero(roiThresh_1);

            //std::cout << "dx: " << abs(dx) << "\n";
            //std::cout << "photo.radius" << Photoball.radius*1.7 << "\n";
            if ((movementPixels_1 > (diameter_p * diameter_p) * 0.1) && (Photoball.active == true)) {

                if (abs(dx) > Photoball.radius * 1.7) {

                    std::cout << "터치! 점수: " << --score << "\n";
                    ballsound(200);
                    Photoball.radius = rand() % 20 + 15; // 10 ~ 29
                    Photoball.position = getRandomPosition(width, height, Photoball.radius);
                    Photoball.respawn = clock();  // 터치 시간 저장
                    
                    if (flip_ran != 2) {
                        cv::flip(Photoball.photo, Photoball.photo, flip_ran);
                    }
                    // 사진 플립
                    flip_ran = rand() % 4 - 1; // -1, 0, 1, 2
                     
                    if (flip_ran != 2) {
                        cv::flip(Photoball.photo, Photoball.photo, flip_ran);
                    }
                }

                else {

                    std::cout << "터치! 점수: " << ++score << "\n";
                    ballsound(1000);
                    Photoball.radius = rand() % 20 + 15; // 10 ~ 29
                    Photoball.position = getRandomPosition(width, height, Photoball.radius);
                    Photoball.respawn = clock();  // 터치 시간 저장
                    flip_ran = rand() % 4 - 1; // -1 , 0 , 1 , 2
                    // 사진 플립
                    if (flip_ran != 2) {
                        cv::flip(Photoball.photo, Photoball.photo, flip_ran);
                    }
                

                }

                // 축이동 원상태로 돌아옴
                dx = 0; dy = 0;
                cv::Mat M(2, 3, CV_32F, data);
                cv::Mat translated_frame;
                cv::warpAffine(ball_img, translated_frame, M, ball_img.size());
                translated_frame.copyTo(frame(ballRect_1), mask);

                
                Photoball.active = false;
                continue; // 위치가 바뀌었으므로 이번 프레임 그리기는 건너뜀
            }

            
        }


        clock_t curren_time_color = clock();
        clock_t diff_time_color = curren_time_color - Ran_Color_Ball.color_t;
        double second_color = diff_time_color / CLOCKS_PER_SEC;

        // 1초마다 색 변경
        if (second_color >= 1) {
            Ran_Color_Ball.color_t = clock();
            Ran_Color_Ball.blue = rand() % 256;
            Ran_Color_Ball.green = rand() % 256;
            Ran_Color_Ball.red = rand() % 256;
        }

        // ROI가 화면 내부에 있을 때만 처리
        if ((ballRect_2 & cv::Rect(0, 0, width, height)) == ballRect_2) {
            // 충돌 감지 (움직임 확인)

            cv::Mat roiThresh_2 = thresh(ballRect_2);
            int movementPixels_2 = cv::countNonZero(roiThresh_2);
            if ((movementPixels_2 > (diameter_r * diameter_r) * 0.1) && (Ran_Color_Ball.active == true)) {
                color_score = (Ran_Color_Ball.red * 0.299) +
                    (Ran_Color_Ball.green * 0.587) +
                    (Ran_Color_Ball.blue * 0.114);

                if (color_score >= 127) {
                    std::cout << " c_s>>> " << color_score << "\n";
                    std::cout << " score>>> " << (color_score / 32) - 3 << "\n";
                    score = score + (color_score / 32) - 3;
                    std::cout << "컬러볼 터치! 점수: " << score << "\n";
                    ballsound(1000);
                }
                else {
                    std::cout << " c_s>>> " << color_score << "\n";
                    std::cout << " score>>> " << (color_score / 32) - 4 << "\n";
                    score = score + (color_score / 32) - 4;
                    std::cout << "컬러볼 터치! 점수: " << score << "\n";
                    ballsound(200);
                }

                Ran_Color_Ball.radius = rand() % 20 + 15; // 10 ~ 29
                Ran_Color_Ball.position = getRandomPosition(width, height, Ran_Color_Ball.radius);
                Ran_Color_Ball.respawn = clock();  // 터치 시간 저장

                Ran_Color_Ball.active = false;
                continue; // 위치가 바뀌었으므로 이번 프레임 그리기는 건너뜀
            }
            // 화면에 score 표기
            cv::putText(frame, "Score: " + std::to_string(score), cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

            cv::imshow("GAME", frame);
            gray_frame.copyTo(prev_gray);
            if (cv::waitKey(10) == 27) break; // esc 종료 

        }


    }// while end
    cap.release();
    cv::destroyAllWindows();
}
