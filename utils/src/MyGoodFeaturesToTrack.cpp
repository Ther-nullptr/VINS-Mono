#include "../include/MyGoodFeaturesToTrack.h"

using namespace cv;
using namespace std;

void my_goodFeaturesToTrack(cv::InputArray _image, cv::OutputArray _corners, int maxCorners, double qualityLevel, double minDistance, cv::InputArray _mask, int blockSize){ // TODO blocksize=3
    Mat image = _image.getMat(), mask = _mask.getMat();
    
    //对参数有一些基本要求
    CV_Assert( qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0 ); 
    CV_Assert( mask.empty() || (mask.type() == CV_8UC1 && mask.size() == image.size()) );
    Mat tmp;

    //TODO: my_func
    // Sobel
    Mat x_grad, y_grad;
    int h, w;
    Sobel(image, x_grad, CV_16S, 1, 0, 3);
	Sobel(image, y_grad, CV_16S, 0, 1, 3);

	w = x_grad.cols;
    h = x_grad.rows;
    for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			x_grad.at<int16_t>(i, j) = x_grad.at<int16_t>(i, j) >> 3;
			y_grad.at<int16_t>(i, j) = y_grad.at<int16_t>(i, j) >> 3;
		}
	}

	Mat xcov = x_grad.mul(x_grad);
	Mat ycov = y_grad.mul(y_grad);
	Mat rcov = x_grad.mul(y_grad);
    
	Mat mkernel = Mat::ones(blockSize, blockSize, CV_16S);
    Mat X_Cov, Y_Cov, R_Cov;

	filter2D(xcov, X_Cov, -1, mkernel);
	filter2D(ycov, Y_Cov, -1, mkernel);
	filter2D(rcov, R_Cov, -1, mkernel);

    Mat eig(X_Cov.size(), CV_32F);
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            eig.at<float>(i,j) = minEig(X_Cov.at<int16_t>(i,j), Y_Cov.at<int16_t>(i,j), R_Cov.at<int16_t>(i,j));
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < w; j++) {
            eig.at<float>(i,j) = 0;
        }
    }
    for (int i = h-2; i < h; i++) {
        for (int j = 0; j < w; j++) {
            eig.at<float>(i,j) = 0;
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < h; j++) {
            eig.at<float>(j,i) = 0;
        }
    }
    for (int i = w-2; i < w; i++) {
        for (int j = 0; j < h; j++) {
            eig.at<float>(j,i) = 0;
        }
    }

    double maxVal = 0;
    minMaxLoc( eig, 0, &maxVal, 0, 0, mask );   //maxVal保存了eig的最大值
    threshold( eig, eig, maxVal*qualityLevel, 0, THRESH_TOZERO );
    dilate( eig, tmp, Mat());  //tmp中保存了膨胀之后的eig
    Size imgsize = image.size(); 
    vector<const float*> tmpCorners;
    for( int y = 1; y < imgsize.height - 1; y++ )
    {
        const float* eig_data = (const float*)eig.ptr(y);  //获得eig第y行的首地址
        const float* tmp_data = (const float*)tmp.ptr(y);  //获得tmp第y行的首地址
        const uchar* mask_data = mask.data ? mask.ptr(y) : 0;
 
        for( int x = 1; x < imgsize.width - 1; x++ )
        {
            float val = eig_data[x];
            if( val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]) )  //val == tmp_data[x]说明这是局部极大值
                tmpCorners.push_back(eig_data + x);  //保存其位置
        }
    }
 
    sort( tmpCorners.begin(), tmpCorners.end(), greaterThanPtr<float>() ); 
    vector<Point2f> corners;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;
 
	if(minDistance >= 1)  
    {
         // Partition the image into larger grids
        int w = image.cols;
        int h = image.rows;
 
        const int cell_size = cvRound(minDistance); 
 
        const int grid_width = (w + cell_size - 1) / cell_size; 
        const int grid_height = (h + cell_size - 1) / cell_size;
 
        std::vector<std::vector<Point2f> > grid(grid_width*grid_height); 
 
        minDistance *= minDistance; 
 
        for( i = 0; i < total; i++ )   
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.data); 
            int y = (int)(ofs / eig.step);  
            int x = (int)((ofs - y*eig.step)/sizeof(float));  
 
            bool good = true; 
 
            int x_cell = x / cell_size;  
            int y_cell = y / cell_size;
 
            int x1 = x_cell - 1;  
            int y1 = y_cell - 1;  
            int x2 = x_cell + 1;  
            int y2 = y_cell + 1;
 
            x1 = std::max(0, x1); 
            y1 = std::max(0, y1);
            x2 = std::min(grid_width-1, x2);
            y2 = std::min(grid_height-1, y2);
 
			for( int yy = y1; yy <= y2; yy++ ) 
            {
                for( int xx = x1; xx <= x2; xx++ ) 
                {
                    vector <Point2f> &m = grid[yy*grid_width + xx]; 
 
                    if( m.size() ) 
                    {               
                        for(j = 0; j < m.size(); j++) 
                        {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;
                            if( dx*dx + dy*dy < minDistance )
                            {                              							
				good = false;
                                goto break_out;
                            }
                        }
                    }
                } 
            } 
 
            break_out:
 
            if(good)
            {
                grid[y_cell*grid_width + x_cell].push_back(Point2f((float)x, (float)y));
 
                corners.push_back(Point2f((float)x, (float)y));
                ++ncorners;
 
                if( maxCorners > 0 && (int)ncorners == maxCorners )  
                    break;
            }
        }
    }
    else
    {
        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.data);
            int y = (int)(ofs / eig.step);  
            int x = (int)((ofs - y*eig.step)/sizeof(float)); 
 
            corners.push_back(Point2f((float)x, (float)y));
            ++ncorners;
            if( maxCorners > 0 && (int)ncorners == maxCorners )  
                break;
        }
    }
 
    Mat(corners).convertTo(_corners, _corners.fixedType() ? _corners.type() : CV_32F);
}

float minEig(int16_t a, int16_t b, int16_t c){ // TODO
    float tmp, sq;
    tmp = a + b;
    sq = sqrt((a-b)*(a-b)+4*c*c);
    return (tmp-sq)/2;
}