#include "main.hpp"

ofstream outFile("angle.txt",ios::out);

#ifdef VIEW_POINTCLOUD
    Viz3d myWindow("Viz Demo");
    Viz3d myWindow2("Viz Demo2");
#endif

int main(int argc, char * argv[])
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    
    initialze();
    
    while (1)//waitKey(1) < 0 && cvGetWindowHandle(window_name))
    {
        rs2::frameset data = rs2_pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = color_map(data.get_depth_frame());
        rs2::frame ir = data.get_infrared_frame();
        auto depth_pc = data.get_depth_frame();
        
        points = pc.calculate(depth_pc);
        
        auto vertices = points.get_vertices();
        
        
        
        vector<point3f> m_PointCloud;
        
        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image_depth(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        Mat image_ir_origin(Size(w, h), CV_8UC1, (void*)ir.get_data(), Mat::AUTO_STEP);
        Mat image_ir = Mat::zeros(Size(w, h), CV_8UC1);
        Mat mat_floor = Mat::zeros(Size(w, h), CV_8UC1);
        
        vector<point3f> temp_floor_points;
        vector<point3f> floor_points;
        
            
        for(int cnt=0;cnt<points.size();cnt++)
        {
            if(vertices[cnt].y < 0.4 && vertices[cnt].y > 0.2)
            {
                point3f m_tempPointCloud;
                point3f m_tempDepthPointCloud;
                point2d temp_pixel_point;               
                
                m_tempDepthPointCloud.x = vertices[cnt].x;
                m_tempDepthPointCloud.y = vertices[cnt].y;
                m_tempDepthPointCloud.z = vertices[cnt].z;
                
                m_tempPointCloud.x = (float)(1000 * vertices[cnt].z);               
                m_tempPointCloud.y = -(float)(1000 * vertices[cnt].x);
                m_tempPointCloud.z = -(float)(1000 * vertices[cnt].y);
                
                temp_pixel_point.x = (int)((m_cam_param.f *  vertices[cnt].x / vertices[cnt].z) + m_cam_param.cx);
                temp_pixel_point.y = (int)((m_cam_param.f *  vertices[cnt].y / vertices[cnt].z) + m_cam_param.cy);
                
                if(temp_pixel_point.x>=0 && temp_pixel_point.x<w && temp_pixel_point.y>=0 && temp_pixel_point.y<h)
                {
                    mat_floor.at<uchar>(temp_pixel_point.y,temp_pixel_point.x) = image_ir_origin.at<uchar>(temp_pixel_point.y,temp_pixel_point.x);
                    m_lineDepth[temp_pixel_point.x][temp_pixel_point.y] = m_tempDepthPointCloud;
                    temp_floor_points.push_back(m_tempPointCloud);
                }
                             
                m_PointCloud.push_back(m_tempPointCloud);
                
               
            }                               
        }
        
        floor_points = computeRANSAC(temp_floor_points);

        //cv::imshow("floor_image",mat_floor);
        
        Mat mat_floorMap = Mat::zeros(FLOORMAP_H,FLOORMAP_W,CV_8UC1);//1채널 uchar
        
#ifdef VIEW_FLOORMAP        
        
//        for(int cnt=0;cnt<FLOORMAP_H * 0.1;cnt++)
//        {
//            if(cnt%10 == 0)
//            {
//                line(mat_floorMap, Point(0,10*cnt), Point(FLOORMAP_W,10*cnt), 100, 1);
//            }
//            else
//            {
//                line(mat_floorMap, Point(0,10*cnt), Point(FLOORMAP_W,10*cnt), 30, 0.5);
//            }         
//        }
//        
//        for(int cnt=0;cnt<FLOORMAP_W * 0.1;cnt++)
//        {
//            if(cnt%10 == 0)
//            {
//                line(mat_floorMap, Point(10*cnt,0), Point(10*cnt,FLOORMAP_H), 100, 1);
//            }
//            else
//            {
//                line(mat_floorMap, Point(10*cnt,0), Point(10*cnt,FLOORMAP_H), 30, 0.5);
//            }         
//        }
        
        
#endif        
        
        Mat contours;
        Mat contours_inv;
        Mat thresh;
        Mat mat_lines = Mat::zeros(Size(w, h), CV_8UC1);
        vector<point3f> line_depths;
        Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11), cv::Point(1, 1));
        morphologyEx(image_ir_origin, image_ir, cv::MorphTypes::MORPH_OPEN, mask);
        //adaptiveThreshold(image_ir, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 101, 10);
        //threshold(image_ir, thresh, 0, 255, THRESH_BINARY | THRESH_OTSU);
        threshold(image_ir,thresh,150,255,THRESH_BINARY);
        Canny(image_ir,contours,125,350,3);
        
        vector<Vec4i> linesP;
        HoughLinesP(contours, linesP, 1, CV_PI/180, 20, 100, 50 ); // runs the actual detection

        for(vector<Vec4i>::const_iterator it=linesP.begin();it!=linesP.end();it++)
        {
            line(mat_lines,Point((*it)[0],(*it)[1]),Point((*it)[2],(*it)[3]),Scalar(255),1);
        }
        
        for(int w_cnt=0;w_cnt<w;w_cnt++)
        {
            for(int h_cnt=0;h_cnt<h;h_cnt++)
            {
                if(mat_lines.at<uchar>(h_cnt,w_cnt) == 255)//(mat_lines.at<uchar>(h_cnt,w_cnt) == 255)
                {
                    line_depths.push_back(m_lineDepth[w_cnt][h_cnt]);
                }               
            }
        }
        
        for(vector<point3f>::iterator it=line_depths.begin();it!=line_depths.end();it++)
        {
            point3d m_tempPointCloud;
            
            m_tempPointCloud.x = (float)(300 * (*it).z);               
            m_tempPointCloud.y = -(float)(300 * (*it).x);
            m_tempPointCloud.z = -(float)(1000 * (*it).y);
            
            m_tempPointCloud.x = (int)m_tempPointCloud.x;
            m_tempPointCloud.y = (int)(m_tempPointCloud.y + FLOORMAP_W * 0.5);
            m_tempPointCloud.z = (int)m_tempPointCloud.z + 300;
            
            if(m_tempPointCloud.x >= 0 && m_tempPointCloud.x < FLOORMAP_H && m_tempPointCloud.y >= 0 && m_tempPointCloud.y < FLOORMAP_W && m_tempPointCloud.z >=0 && m_tempPointCloud.z < 255)
                mat_floorMap.at<uchar>(m_tempPointCloud.x,m_tempPointCloud.y) = m_tempPointCloud.z;
        }
        
        vector<point2d> cell_line_depths;
        
        for(int w_cnt=0;w_cnt<FLOORMAP_W;w_cnt++)
        {
            for(int h_cnt=0;h_cnt<FLOORMAP_H;h_cnt++)
            {
                if(mat_floorMap.at<uchar>(h_cnt,w_cnt) != 0)
                {
                    point2d temp_cell_line_depths;
                    temp_cell_line_depths.x = w_cnt;
                    temp_cell_line_depths.y = h_cnt;
                    cell_line_depths.push_back(temp_cell_line_depths);
                }               
            }
        }
        
#ifdef VIEW_FLOORMAP
        flip(mat_floorMap, mat_floorMap, -1); // Rotate H/V
        Mat mat_colorFloorMap;
        applyColorMap(mat_floorMap, mat_colorFloorMap, COLORMAP_JET);
        //cv::imshow("FloorMap",mat_colorFloorMap);
#endif
        

        Mat win_mat(Size(1280, 720), CV_8UC1);
        Mat win_mat_1(Size(1280, 720), CV_8UC1);
        resize( mat_lines, mat_lines, Size(640, 360), 0, 0, CV_INTER_LINEAR );
        resize( image_ir, image_ir, Size(640, 360), 0, 0, CV_INTER_LINEAR );
        resize( thresh, thresh, Size(640, 360), 0, 0, CV_INTER_LINEAR );
        resize( mat_floor, mat_floor, Size(640, 360), 0, 0, CV_INTER_LINEAR );
        resize( contours, contours, Size(640, 360), 0, 0, CV_INTER_LINEAR );
        resize( mat_floorMap, mat_floorMap, Size(640, 360), 0, 0, CV_INTER_LINEAR );
        resize( image_ir_origin, image_ir_origin, Size(640, 360), 0, 0, CV_INTER_LINEAR );
        // Copy small images into big mat
        hconcat(image_ir_origin, contours, win_mat); // horizontal
        hconcat(win_mat, mat_floor, win_mat); // horizontal
        //hconcat(contours, mat_floor, win_mat_1);
        hconcat(image_ir, mat_lines, win_mat_1);
        hconcat(win_mat_1, mat_floorMap, win_mat_1);
        vconcat(win_mat, win_mat_1, win_mat);

        // Update the window with new data
        //cv::imshow("DEPTH", image_depth);
        cv::imshow("WINDOW",win_mat);
        //imshow("IR",image_ir);
        //imshow("CONTOURS",contours_inv);
        
        
        waitKey(1);
        
        float angle;
        static float sum_angle = 0;
        static vector<point2d> prev_line_points;
        
        
        if(cell_line_depths.size() > 10 && prev_line_points.size()>0)
        {

             angle = point2planeICP(cell_line_depths,prev_line_points);
             //printf("angle = %f\n",angle);
             outFile<<angle<<endl;

//            if(abs(angle)<5)
//            {
//                float filtered_angle = kalmanFilter(angle);
//                sum_angle += filtered_angle;                
//                printf("sum = %f, angle = %f, filtered_angle = %f\n",sum_angle,angle, filtered_angle);
//            }                      
        }
        
        prev_line_points = cell_line_depths;
    }

    return EXIT_SUCCESS;
}

void initializeRS2(void)
{
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    // Start streaming with default recommended configuration
    rs2_pipe.start(cfg);
}

void initializeCamParams(void)
{
    m_cam_param.cx = 644.113;
    m_cam_param.cy = 350.399;
    m_cam_param.f = 646.172;
}

float point2planeICP(vector<point2d> curr_line_points, vector<point2d> prev_line_points) {

    int32_t dim = 2;
    int32_t num_curr = curr_line_points.size();
    int32_t num_prev = prev_line_points.size();

    //    printf("num_curr = %d, num_prev = %d\n",num_curr,num_prev);


    double* M = (double*) calloc(2 * num_curr, sizeof (double));
    double* T = (double*) calloc(2 * num_prev, sizeof (double));

    int32_t k = 0;
    int32_t j = 0;

    for (vector<point2d>::iterator it = curr_line_points.begin(); it != curr_line_points.end(); it++) {
        M[k * 2 + 0] = (double) ((*it).x);
        M[k * 2 + 1] = -(double) ((*it).y);
        //M[k * 3 + 2] = -(double) (1000 * (*it).y);
        k++;
    }

    for (vector<point2d>::iterator it = prev_line_points.begin(); it != prev_line_points.end(); it++) {
        T[j * 2 + 0] = (double) ((*it).x);
        T[j * 2 + 1] = -(double) ((*it).y);
        //T[j * 3 + 2] = -(double) (1000 * (*it).y);
        j++;
    }

    // start with identity as initial transformation
    // in practice you might want to use some kind of prediction here
    Matrix R = Matrix::eye(2);
    Matrix t(2, 1);

    // run point-to-plane ICP (-1 = no outlier threshold)
    //cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
    //IcpPointToPoint icp(M, num_curr, dim);
    IcpPointToPlane icp(M, num_curr, dim);
    double residual = icp.fit(T, num_prev, R, t, 0);

    float angle = asin(R.val[1][0]) * RAD2DEG;
    //printf("angle = %f\n",angle);
//
//            cout << endl << "Transformation results:" << endl;
//            cout << "R:" << endl << R << endl << endl;
//            cout << "t:" << endl << t << endl << endl;
//            cout << "Residual:"<<residual;

    // free memory
    free(M);
    free(T);

    return angle;


    // allocate model and template memory

}

float kalmanFilter(float curr_angle)
{
    static int isFirst = 0;
    static float P = 6;
    float filtered_angle;
    
    int A = 1;
    int H = 1;
   
    int Q = 0;
    int R = 1;
    int x = 0;
    
    float xp = A * x;
    float Pp = A * P * A + Q;
    float K = Pp * H / (H * Pp * H + R);
    filtered_angle = xp + K * (curr_angle - H * xp);
    P = Pp - K * H * Pp;
    return filtered_angle;
}

void initUart(void) {
    struct termios newtio;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Check Device or SuperUser Permission\n");
        exit(-1);
    }

    memset(&newtio, 0, sizeof (newtio));

    newtio.c_cflag = B115200;
    newtio.c_cflag |= CS8;
    newtio.c_cflag |= CLOCAL;
    newtio.c_cflag |= CREAD;
    newtio.c_iflag = IGNPAR;
//  newtio.c_iflag = ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    usleep(100);

    //close(fd);
}

void initializeThread(void)
{
    pthread_t dynamixel_ctr;
    pthread_create(&dynamixel_ctr, NULL, &thread_dynamixel, 0);
}

void initialze(void)
{
    initializeRS2();
    initializeCamParams();
#ifdef USE_DINAMIXEL
    initUart();
    initializeThread();
#endif
}

void *thread_dynamixel(void *arg) {
    
    int goal_position = 180;
    int direction = 1;
    
    while (1) {
        
        servo(1, (int)(goal_position / ANGLE_ENCODER_RATIO), 300);
        
        goal_position = goal_position+(direction * 1);
        
        if(goal_position>=220)
        {
            direction = -1;
        }else if(goal_position <= 140)
        {
            direction = 1;
        }
        usleep(20000);
    }
    

    pthread_exit((void *) 0);
}

unsigned int servo(unsigned char ID,unsigned int Position,unsigned int Speed){
    
    unsigned char Instruction_Packet_Array[11];
    
    Instruction_Packet_Array[0] = HEADER;
    Instruction_Packet_Array[1] = HEADER;
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = SERVO_GOAL_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[6] = (unsigned char)Position;
    Instruction_Packet_Array[7] = (unsigned char)((Position & 0x0F00) >> 8);
    Instruction_Packet_Array[8] = (unsigned char)Speed;
    Instruction_Packet_Array[9] = (unsigned char)((Speed & 0x0F00) >> 8);

    unsigned int checksum_packet = Instruction_Packet_Array[2] + Instruction_Packet_Array[3] + Instruction_Packet_Array[4];

    for (unsigned char i = 5; i <= Instruction_Packet_Array[3] + 2; i++){      
        checksum_packet += Instruction_Packet_Array[i];
    }
    
    Instruction_Packet_Array[10] = ~checksum_packet & 0xFF;
    
    write(fd,Instruction_Packet_Array,sizeof(Instruction_Packet_Array));
    
    return 0;
}

plane_t computePlane(point3f p1,point3f p2,point3f p3)
{
    mat3x3_t mat_temp;
    plane_t plane;
    
    mat_temp.data[0][0] = 1;
    mat_temp.data[0][1] = p1.y;
    mat_temp.data[0][2] = p1.z;
    mat_temp.data[1][0] = 1;
    mat_temp.data[1][1] = p2.y;
    mat_temp.data[1][2] = p2.z;
    mat_temp.data[2][0] = 1;
    mat_temp.data[2][1] = p3.y;
    mat_temp.data[2][2] = p3.z;
    
    plane.a = det(mat_temp);
    
    mat_temp.data[0][0] = p1.x;
    mat_temp.data[0][1] = 1;
    mat_temp.data[0][2] = p1.z;
    mat_temp.data[1][0] = p2.x;
    mat_temp.data[1][1] = 1;
    mat_temp.data[1][2] = p2.z;
    mat_temp.data[2][0] = p3.x;
    mat_temp.data[2][1] = 1;
    mat_temp.data[2][2] = p3.z;
    
    plane.b = det(mat_temp);
    
    mat_temp.data[0][0] = p1.x;
    mat_temp.data[0][1] = p1.y;
    mat_temp.data[0][2] = 1;
    mat_temp.data[1][0] = p2.x;
    mat_temp.data[1][1] = p2.y;
    mat_temp.data[1][2] = 1;
    mat_temp.data[2][0] = p3.x;
    mat_temp.data[2][1] = p3.y;
    mat_temp.data[2][2] = 1;
    
    plane.c = det(mat_temp);
    
    mat_temp.data[0][0] = p1.x;
    mat_temp.data[0][1] = p1.y;
    mat_temp.data[0][2] = p1.z;
    mat_temp.data[1][0] = p2.x;
    mat_temp.data[1][1] = p2.y;
    mat_temp.data[1][2] = p2.z;
    mat_temp.data[2][0] = p3.x;
    mat_temp.data[2][1] = p3.y;
    mat_temp.data[2][2] = p3.z;
    
    plane.d = -det(mat_temp);
    
    return plane;  
}

float det(mat3x3_t matrix)
{
    return matrix.data[0][0]*(matrix.data[1][1]*matrix.data[2][2]-matrix.data[1][2]*matrix.data[2][1])
          -matrix.data[0][1]*(matrix.data[1][0]*matrix.data[2][2]-matrix.data[1][2]*matrix.data[2][0])
          +matrix.data[0][2]*(matrix.data[1][0]*matrix.data[2][1]-matrix.data[1][1]*matrix.data[2][0]);
}

double distPoint2Plane(point3f point, plane_t plane)
{
    float dist = abs(plane.a*point.x+plane.b*point.y+plane.c*point.z+plane.d) / sqrt(plane.a*plane.a + plane.b*plane.b + plane.c*plane.c);
    return dist;
}

vector<point3f> computeRANSAC(vector<point3f> floorpoints)
{
    vector<point3f> result;
    int max_iter = 10;
    int iter=0;
    int i0,i1,i2;
    int N = floorpoints.size();
    point3f p1,p2,p3;
    plane_t plane;
    ransac_t ransac;
    ransac_t temp_ransac;
    double dist_th = 5.;
    int num_th = 0.8 * floorpoints.size();
    
    printf("floorpoints size = %d",floorpoints.size());
    
    srand(unsigned(time(0)));
    
    ransac.num = 0;
    
    while(iter<max_iter)
    {
        temp_ransac.point.clear();
        temp_ransac.dist = 0;
        temp_ransac.num = 0;
        result.clear();
        
        i0 = rand()%N;
        result.push_back(floorpoints[i0]);
        do{
            i1 = rand()%N;
        }while(i1 == i0);
        result.push_back(floorpoints[i1]);
        do{
            i2 = rand()%N;
        }while(i1 == i0 || i2 == i1 || i2 == i0);
        result.push_back(floorpoints[i2]);
        
        p1 = result[0];
        p2 = result[1];
        p3 = result[2];
        
        plane = computePlane(p1,p2,p3);
        
        int n=3;
        
        for(int j=0;j<N;j++)
        {
            if(j==i0 || j==i1 || j==i2)continue;
            double dist = distPoint2Plane(floorpoints[j],plane);
            if(dist <= dist_th)
            {
                result.push_back(floorpoints[j]);
                temp_ransac.dist += dist;
                temp_ransac.num += 1;
            }
        }
        temp_ransac.point = result;
        temp_ransac.plane = plane;
        
        if(temp_ransac.num > ransac.num)
        {
            ransac = temp_ransac;
            printf("update ransac!!!!!\n");
        }
        
        iter++;
    }
    
    printf(" rasac_num = %d iter = %d\n",ransac.num,iter);
    
    double min_dist = dist_th * N;
    int min_plane_num = 0;
    int max_num = 0;
    
    Mat mat_point_cloud = Mat::zeros(1, ransac.point.size(), CV_32FC3);
    Mat mat_point_cloud_raw = Mat::zeros(1, floorpoints.size(), CV_32FC3);
    
    #ifdef VIEW_POINTCLOUD 

    int pc_cnt=0;
        
            for(vector<point3f>::const_iterator it = floorpoints.begin();it!=floorpoints.end();++it)
            {                     
                mat_point_cloud_raw.at<Vec3f>(0,pc_cnt)[0] = (*it).x;
                mat_point_cloud_raw.at<Vec3f>(0,pc_cnt)[1] = (*it).y;
                mat_point_cloud_raw.at<Vec3f>(0,pc_cnt)[2] = (*it).z;
                pc_cnt++;
            }
    pc_cnt=0;
    
    for(vector<point3f>::const_iterator it = ransac.point.begin();it!=ransac.point.end();++it)
            {                     
                mat_point_cloud.at<Vec3f>(0,pc_cnt)[0] = (*it).x;
                mat_point_cloud.at<Vec3f>(0,pc_cnt)[1] = (*it).y;
                mat_point_cloud.at<Vec3f>(0,pc_cnt)[2] = (*it).z;
                pc_cnt++;
            }
        
        
    #endif
    
    #ifdef VIEW_POINTCLOUD        
       // WCloud cloud_widget = WCloud(mat_point_cloud, Color::green());
       // cloud_widget.setRenderingProperty(POINT_SIZE, 1 );
        
        
        WCloud cloud_widget1 = WCloud(mat_point_cloud_raw, Color::green());
        myWindow.showWidget("Depth1", cloud_widget1);
        WCloud cloud_widget2 = WCloud(mat_point_cloud, Color::blue());
        myWindow2.showWidget("Depth2", cloud_widget2);
        //WCloud cloud_widget3 = WCloud(mat_point_cloud[2], Color::red());
        //myWindow.showWidget("Depth3", cloud_widget3);
        
        myWindow.spinOnce( 1, true );  
        myWindow2.spinOnce( 1, true);
    #endif
    
    return ransac.point;
}