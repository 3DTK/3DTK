#include "segmentation/graph_cut/graph_cut.h"
#include "segmentation/graph_cut/util.h"
#include "segmentation/graph_cut/david_graph.h"
#include "segmentation/graph_cut/blob_color.h"

#include <slam6d/fbr/panorama.h>
#include <slam6d/fbr/projection.h>
#include <slam6d/fbr/fbr_global.h>

#include <opencv2/core/core.hpp>

#include <newmat/newmatap.h>

#include <boost/math/distributions/normal.hpp>
#include <boost/program_options.hpp>

#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>

using fbr::scan_cv;
double epsilon = 0.0001;

struct weight{
    using normal_dist = boost::math::normal_distribution<double>;
    normal_dist m_distribution;
    bool degenerated;
    weight(double sum, double squared_sum, int count)
    {
        //estimate normal distribution of distances
        double mean = sum / count;
        double sigma;
        if(count > 1) {
            sigma = std::sqrt((squared_sum - 2 * mean * sum + count * mean * mean) / (count - 1));
            m_distribution = normal_dist(mean, sigma);
            degenerated = false;

        } else {
            degenerated = true;
        }
        
    }
    double operator()(double distance) const{
        return !degenerated ? 1 - boost::math::cdf(m_distribution, distance) : 1;
    }
};

//Internal use for testing
std::vector<segment> segment_scan(scan_cv& scan, size_t panorama_width, size_t panorama_height, size_t window_size,
                                  size_t minimum_points, double tau, double cell_size, bool disable_post_processing);
std::vector<segment> segment_scan(scan_cv& scan, size_t panorama_width, size_t panorama_height, size_t window_size,
                                  size_t minimum_points, double tau, double cell_size)
{
    return segment_scan(scan, panorama_width, panorama_height, window_size, minimum_points, tau, cell_size, false);
}

int main(int argc, char** argv)
{   /*TODO
    * maybe projection_method
    */
    namespace po = boost::program_options;
    po::options_description options("Options");
    options.add_options()
        ("help,?", "print help message")
        ("window,m", po::value<size_t>()->default_value(5), "moving window size")
        ("numpoints,n", po::value<size_t>()->default_value(5), "minimum number of points for a plane candidate to be considered")
        ("tau,t", po::value<double>()->default_value(0.6), "planarity threshold")
        ("single,s", po::bool_switch()->default_value(false), "write labels in a single file using different colors");
        
    po::options_description panorama_options("Panorama Options");
    panorama_options.add_options()
        ("height,h",po::value<size_t>()->required(), "maximum internal panorma height")
        ("width,w",po::value<size_t>()->required(), "maximum internal panorma width");
        
    po::positional_options_description input_dir;
    input_dir.add("input-dir", 1);
    po::options_description dummy("Hidden");
    dummy.add_options()("input-dir",po::value<std::string>()->required(), "input_directory");
    
    po::options_description pp_options("Post processing options") ;
    pp_options.add_options()("cellsize,c",po::value<double>()->default_value(10.0), "Bin size for blob coloring during post processing");
    dummy.add_options()("disable-pp", po::bool_switch()->default_value(false)); //Leave this undocumented
    
    po::options_description visible;
    visible.add(options).add(panorama_options).add(pp_options);
    po::options_description all;
    all.add(visible).add(dummy);
    po::variables_map vm;
    

    try {
        po::store(po::command_line_parser(argc, argv).options(all).positional(input_dir).run(), vm);
        if(vm.count("help") || argc == 1 || !vm.count("input-dir")) {
            std::cout << "Usage: " << argv[0] << " options input_directory\n";
            std::cout << visible << "\n";
            return 0;
        }
        po::notify(vm);
    }
    catch(exception& e) {
        std::cout << e.what() << "\n" << visible;
        return 0;
    }
#ifndef NDEBUG
    std::cout << "m=" << vm["window"].as<size_t>() << "\n"
        << "w=" << vm["width"].as<size_t>() << "\n"
        << "h=" << vm["height"].as<size_t>() << "\n"
        << "dir=" << vm["input-dir"].as<std::string>() << "\n"
        << "tau=" << vm["tau"].as<double>() << "\n"
        << "n=" << vm["numpoints"].as<size_t>() << "\n"
        << "c=" << vm["cellsize"].as<double>() << "\n";
#endif
    scan_cv scan{vm["input-dir"].as<string>(), 0, UOS};
    scan.convertScanToMat();
    auto segments  = segment_scan(scan, vm["width"].as<size_t>(), vm["height"].as<size_t>(), vm["window"].as<size_t>(),
            vm["numpoints"].as<size_t>(), vm["tau"].as<double>(), vm["cellsize"].as<double>(), vm["disable-pp"].as<bool>());

    std::string out_path = "segments";
    std::string  out_path_base = out_path;
    int i = 1;
    while(mkdir(out_path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO) == -1) {
        if(errno == EEXIST){
            std::cout << "Directory \"" << out_path << "\" already exists! Trying to write to \"";
            out_path = out_path_base + std::to_string(i);
            std::cout << out_path << '\"' << std::endl;
            ++i;
        }
        else{
            std::cout << "Error creating directory \"" << out_path << "\"! Exiting... \n";
            return 1;
        }
    }
    std::cout << "Writing segments to " << out_path << std::endl;
    if(!vm["single"].as<bool>()){
         write_scan(segments, out_path, 0,[](const segment& s){return "n:"+cvVecnT_string(s.normal)+" d:"+std::to_string(s.distance);},
               identity(), [](const segment& s){return s.points;});
    } else {
        segments.erase(segments.begin());
        write_single_color_scan(segments, out_path, 0, identity(), [](const segment& s){return s.points;});
    }
    return 0;
}

void similarity_measure(const cv::Mat& range_image, int window_size, std::vector<std::vector<segment>>& phis)
{   
#ifndef NDEBUG
    int ncv = 0;
    std::ofstream normals_debug("debug/normals/normals.txt");
    std::ofstream normals_deviations("debug/normals/deviations.txt");
    //std::cout << "(row, col) point origin normal \n";
#endif
    int half_size = window_size / 2;
    for(int row = 0; row < range_image.rows; ++row){
        for(int col = 0; col < range_image.cols; ++col){
            const cv::Vec3f& poi = range_image.at<cv::Vec3f>(row, col);
            if(poi == cv::Vec3f{0, 0, 0}){
                //if point is not valid set everything ot zero
                phis[row][col] = segment{{},{0, 0, 0},0};
                continue;
            }
             //investigate region
            double sum = 0, squared_sum = 0;
            cv::Vec3f origin(0, 0, 0);
            int neighbors = 0;
            for(int region_row = row - half_size; region_row <= row + half_size; ++region_row){
                if(region_row < 0 || region_row >= range_image.rows ) continue;
                for(int region_col = col - half_size; region_col <= col + half_size; ++region_col){
                    if((region_col == col && region_row == row)|| region_col < 0 || region_col >= range_image.cols) continue;
                        const cv::Vec3f& xi = range_image.at<cv::Vec3f>(region_row, region_col);
                        if(xi == cv::Vec3f{0, 0, 0}) continue;
                        origin += xi;
                        double distance = cv::norm(xi - poi);
                        sum += distance;
                        squared_sum += distance * distance;
                        ++neighbors;
                }
            } 
            origin = origin * (1 /(float)neighbors);
            if(sum == 0) continue;
            //calculate covariance
            cv::Matx33d covariance = cv::Matx33d::zeros();
            double sum_of_weights = 0;
            weight region_weight(sum, squared_sum, neighbors);
            for(int region_row = row - half_size; region_row <= row + half_size; ++region_row){
                if(region_row < 0 || region_row >= range_image.rows ) continue;
                for(int region_col = col - half_size; region_col <= col + half_size; ++region_col){
                    if((region_col == col && region_row == row)|| region_col < 0 || region_col >= range_image.cols) continue;
                    const cv::Vec3f& xi = range_image.at<cv::Vec3f>(region_row, region_col);
                    if(xi == cv::Vec3f{0, 0, 0}) continue;
                    double distance = cv::norm(xi - poi);
                    double w = region_weight(distance);
                    cv::Vec3f diff(xi - origin);
                    covariance += w * diff * diff.t();
                    sum_of_weights += w;
                }
            }
            
            covariance *= 1 / sum_of_weights;
            //Opencv SVD only calculates left eigenvectors
            NEWMAT::Matrix A(3, 3); A << (double*)(cv::Mat(covariance).data);
            NEWMAT::Matrix U(3, 3); NEWMAT::DiagonalMatrix D(3); NEWMAT::Matrix V(3, 3);
            try{
                NEWMAT::SVD(A, D, U,  V);
            } catch(NEWMAT::ConvergenceException&) {
                phis[row][col] = segment{{}, {0,0,0}, 0};
#ifndef NDEBUG
                ++ncv; 
                continue;
#endif
            }
            /*D is sorted descending and
            Eigenvector to smallest Eigenvalue is last column of V*/
            NEWMAT::Matrix Vt = V.t();
            //cv::Vec3d n(Vt(1,1), Vt(1,2), Vt(1,3));
            int index;
            D.MinimumAbsoluteValue1(index);
            cv::Vec3d n(V(1,index), V(2,index), V(3, index));
            phis[row][col] = segment{{}, n, origin.ddot(n)};
            //std::cout << "(" << row << "," << col << ") " << poi << origin << " " << n << "\n";
            //std::cout << poi << origin << n << "\n";
#ifndef NDEBUG
            normals_debug << n << '\n';
            //if normal is not aligned to a axis
            if((n[0] + n[1] + n[2]) > 1.05 ) normals_deviations << "p:" << poi << "n:" << n << '\n';  
#endif
        } 
    }
#ifndef NDEBUG
    normals_debug.close();
    normals_deviations.close();
    std::cout << "NCE " << ncv/(double)(range_image.rows*range_image.cols) << '\n';
#endif
}


 void edge_distances(const cv::Mat& range_image, const std::vector<std::vector<segment>>& phis, david_graph::grid_graph<double>& graph)
 {
    for(const auto& u : graph.vertices())
    {
        for(const auto& edge : graph.edges(u))
        {
            const std::pair<int, int>& v = edge.first;
            double dist  = fabs(plane_point_distance(range_image.at<cv::Vec3f>(u.first, u.second),phis[v.first][v.second].normal,phis[v.first][v.second].distance))
                + fabs(plane_point_distance(range_image.at<cv::Vec3f>(v.first, v.second),phis[u.first][u.second].normal,phis[u.first][u.second].distance));
            graph.set_weight(u, v, dist);
#ifndef NDEBUG
            static std::ofstream dis("debug/distances/distances.txt");
            dis << dist << '\n';
#endif
        }
    }
}


double standard_error(const david_graph::grid_graph<double>& graph, const cv::Mat& range_image, const cv::Vec3d& normal, double distance)
{
    double sum = 0, squared_sum = 0;
    for(const auto& index : graph.vertices()) {
        double dist = plane_point_distance(range_image.at<cv::Vec3f>(index.first, index.second), normal, distance);
        sum += dist;
        squared_sum += (dist*dist);
    }
    int n = graph.vertices().size();
    double arg = (squared_sum-sum*sum/n)/(n-1);
    return std::sqrt(arg);
}

double standard_error(const plane_candidate& plane)
{
    double sum = 0, squared_sum = 0;
    for(const cv::Vec3f* point : plane.points) {
        double dist = plane_point_distance(*point, plane.normal, plane.distance);
        sum += dist;
        squared_sum += (dist*dist);
    }
    int n = plane.points.size();
    double arg = (squared_sum-sum*sum/n)/(n-1);
    return std::sqrt(arg);
}


std::vector<const cv::Vec3f*> relabel_points(std::vector<plane_candidate>& planes, const std::set<std::pair<int, int>>& indices,
                    const cv::Mat& range_image, double tau)
{
    std::vector<const cv::Vec3f*> non_planar;
    //save sizes
    std::vector<int> sizes(planes.size());
    std::transform(planes.begin(), planes.end(), sizes.begin(), [](const plane_candidate& p){return p.points.size();});
    //clear supporting points
    for_each(planes.begin(), planes.end(), [](plane_candidate& c){ c.points.clear();});
    for(const auto& vertex : indices) {
        double min = std::numeric_limits<double>::max();
        int match = -1;
        const cv::Vec3f* point = &range_image.at<cv::Vec3f>(vertex.first, vertex.second);
        if(cv::norm(*point-cv::Vec3f{97.985, -57.7953, -99.9205}) < epsilon) {
            auto p = cv::Vec3f{97.985, -57.7953, -99.9205}; // 97.985 -57.7953 -99.9205
            p[0] = 1;
            match = -1;
        }
        for(size_t i = 0; i < planes.size(); ++i) {  
            double distance = std::fabs(plane_point_distance(*point, planes[i].normal, planes[i].distance));
            if(distance < min) {
                min = distance;
                match = i;
            } else if(distance == min) {
                match = sizes[i] >sizes[match] ? i : match;
            }
                
        }
        if(min <= std::min(tau, 3 * planes[match].std_error)) {
            planes[match].points.emplace_back(point);
        } else {
            non_planar.emplace_back(point);
        }
    }
    return non_planar;
}

bool refit_planes(std::vector<plane_candidate>& planes) 
{
    bool changed = false;
    for(auto& plane : planes) {
        double new_plane[4];
        double new_error = standard_error(plane);
        calc_plane(plane.points, new_plane);
        if(std::fabs(new_plane[0] - plane.normal[0]) > epsilon || std::fabs(new_plane[1] - plane.normal[1]) > epsilon
            || std::fabs(new_plane[2] - plane.normal[2]) > epsilon || std::fabs(new_plane[3] - plane.distance) > epsilon
            || std::fabs(new_error - plane.std_error) > epsilon){
                changed = true;
        }
        plane.normal = cv::Vec3d{new_plane[0], new_plane[1], new_plane[2]};
        plane.distance = new_plane[3];
        plane.std_error = standard_error(plane);
        
    }
    return changed;
}


void join_planes(std::vector<plane_candidate>& planes, double tau)
{
    //join similar planes
    std::vector<plane_candidate>::iterator end = planes.end();
    for(auto first = planes.begin(); first != end; ++first) {
        for(auto second = first+1; second < end; ++second) {
            if(fabs(first->distance - second->distance) < tau   && acos(first->normal.dot(second->normal)) < M_PI/180) {
#ifndef NDEBUG 
                std::cout << "joining (" << first->normal << ',' << first->distance << ") and (" 
                    << second->normal << ',' << second->distance << ")\n";
#endif
                first->distance = (first->distance + second->distance) / 2;
                first->normal = (first->normal + second->normal) * 0.5;
                first->std_error = (first->std_error + second->std_error);
                //first->points.insert(first->points.begin(), second->points.begin(), second->points.end());
                /* Since second is now joined into first, move it to the end of the vector and dont consider it anymore*/
                --end;
                std::swap(*second, *end);
#ifndef NDEBUG
                std::cout << "First " << first-planes.begin() << " Second " << second-planes.begin()
                    << "End now " << end-planes.begin() << '\n'; 
#endif
            }
        }
    }
    planes.erase(end, planes.end());
}

std::vector<const cv::Vec3f*> k_means(std::vector<plane_candidate>& candidates, const std::set<std::pair<int, int>>& points,
             const cv::Mat& range_image, double tau, size_t minimum_points) 
{
    size_t old_size;
    std::vector<const cv::Vec3f*> non_planar;
    do {
        join_planes(candidates, tau);
        //relabel
        non_planar = relabel_points(candidates, points, range_image, tau);
        old_size = candidates.size();
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
            [=](plane_candidate& c){return c.points.size() < minimum_points;}),  candidates.end());//delete (almost) empty planes
        /*No dilation and joining as described in the paper but with custom joining above*/
    } while (refit_planes(candidates) || old_size != candidates.size()); //refit
  
    return non_planar;
}
        
/* Return plane candidates with non planar points being in the first element */
std::vector<plane_candidate> cut_graph(david_graph::grid_graph<double>& graph, const cv::Mat& range_image, size_t minimum_points, double tau)
{
    /* Calculate threshold used for deleting edges*/
    double sum = 0, squared_sum = 0;
    int num_edges = 0;
    for(const auto& u : graph.vertices()) {
        for(const auto& edge : graph.edges(u)) {
            double distance = edge.second;
            sum += distance;
            squared_sum += distance * distance;
            ++num_edges;
        }
    }
    weight my_weight(sum, squared_sum, num_edges);
    std::vector<double> weights;
    // weights.reserve(num_edges);
    sum = 0;
     for(const auto& u : graph.vertices()) {
        for(const auto& edge : graph.edges(u)) {
            double weight = my_weight(edge.second);
            weights.push_back(weight);
            sum += weight;
        }
    }
    double threshold = sum / num_edges;
    int low, high;
    double mean_low, mean_high;
    double old_threshold;
    do{
        old_threshold = threshold;
        high = low = 0;
        mean_high = mean_low = 0;
        for(double d : weights){
            if(d >= threshold){
                mean_high += d;
                ++high;
            } else {
                mean_low += d;
                ++low;
            }            
        }
        mean_low /= low;
        mean_high /= high;
        threshold = (mean_low + mean_high)/2;
    } while(std::fabs(old_threshold - threshold) > epsilon);
    
    /* remove edges less than thresholdfor_each <=> retain equal or greater*/
    graph.remove_edges([=](std::pair<int, int> , std::pair<int, int> , double distance){
        return my_weight(distance) < threshold;
    });
#ifndef NDEBUG
    std::ofstream after("debug/distances/weights_after.txt");
    std::cout << "Before thresholding:\n" << num_edges << " edges, mean distance " << my_weight.m_distribution.mean() << '\n';
    double sum_filtered = 0;
    int num_filtered = 0;
    for(const auto& u : graph.vertices()) {
        for(const auto& edge : graph.edges(u)) {
            double distance =  edge.second;
            sum_filtered += distance;
            after << my_weight(distance) << '\n';
            ++num_filtered; 
        }
    }
    std::cout << "After thresholding(" << threshold << "):\n" << num_filtered << " edges, mean distance " << sum_filtered/num_filtered << '\n';
    after.close();
#endif
    
    /* Main part, recursively fit planes to the remaining components and employ Kmeans clustering*/
    auto components = graph.connected_components();
#ifndef NDEBUG
    static int i= 0;
    static std::string path = "debug/components";
    int old_i =  i;
    std::string old_path = path;
    path += ('/' + std::to_string(i));
    mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    std::cout << path.substr(16, path.size()) << '\n';
    std::cout << "Found " << components.size() << " components\n";
    write_components(components, path+"/before", minimum_points,
               [&](const std::pair<int, int>& v){return range_image.at<cv::Vec3f>(v.first, v.second);},
               [&](const david_graph::grid_graph<double>& component){return component.vertices();});
    std::ofstream weight_file(path+"/weights.txt");
    for(const double w : weights) weight_file << w << '\n';
    weight_file.close();
    i = 0;
#endif
    /*delete non candidates*/
    components.erase(std::remove_if(components.begin(), components.end(),
        [=](david_graph::grid_graph<double>& component){return component.vertices().size() < minimum_points;}), components.end());
#ifndef NDEBUG
    std::cout << "Considering " << components.size() << " candidates\n";
#endif
    std::vector<plane_candidate> candidates;
    for(auto& candidate : components) {
        double plane[4];
        calc_plane(candidate.vertices(), range_image, plane);
        cv::Vec3d normal(plane[0], plane[1], plane[2]);
        double std_err = standard_error(candidate, range_image, normal, plane[3]);
        if(std_err < tau) {
            /*candidate is planar, no need to insert points, since they would be cleared in the first relabeling step*/
            candidates.push_back({normal, plane[3], std_err, std::vector<const cv::Vec3f*>()});
        } 
        else {
            //recurse
            auto sub_planes = cut_graph(candidate, range_image, minimum_points, tau);
            /*plus one because the points not belonging to any segment are returned in the first element*/
            candidates.insert(candidates.end(), sub_planes.begin()+1, sub_planes.end());
        }
    }
    if(candidates.size() != 0)
    {
#ifndef NDEBUG
        std::cout << path.substr(16, path.size()) << " kmeans" << " (" << graph.vertices().size() << " points, " 
            <<  candidates.size() << " planes)\n";
#endif
        auto non_planar_points = k_means(candidates, graph.vertices(), range_image, tau, minimum_points);
        candidates.insert(candidates.begin(),{{0,0,0},0,0,non_planar_points});
    } else {
        plane_candidate p;
        std::for_each(graph.vertices().cbegin(), graph.vertices().cend(),[&](std::pair<int, int> v)
            {p.points.emplace_back(&range_image.at<cv::Vec3f>(v.first, v.second));} );
        candidates.emplace_back(p);
    }
#ifndef NDEBUG
        std::cout << candidates.size() - 1 << " candidates remaining\n\n";
        write_scan(candidates, path+"/after", 0,
            [](const plane_candidate& p){return "n:"+cvVecnT_string(p.normal)+" d:"+std::to_string(p.distance)+ " sigma:"+std::to_string(p.std_error);},
            [](const cv::Vec3f* p){return *p;}, [](const plane_candidate& p){return p.points;});
#endif

#ifndef NDEBUG
    i = old_i + 1;
    path = old_path;
#endif
    return candidates;
}

/**
 * Not really used right now, returns panorama with the passed dimensions
 * TODO estimate best panorama size depending on some value
 */
fbr::panorama best_size_panorama(scan_cv& scan, size_t max_width, size_t max_height)
{
    using fbr::panorama;
    panorama best;
    double opt = 0;
    int n  = scan.getMatScan().rows * scan.getMatScan().cols;
    for(size_t width=max_width, height=max_height; width > 10 && height > 10; width /= 2, height /= 2) {
        panorama p(width, height, fbr::EQUIRECTANGULAR, 1, 0, fbr::FARTHEST, scan.getZMin(), scan.getZMax(),-90, 90);
        p.createPanorama(scan.getMatScan());
        return p;
        /* Dead code below*/
        cv::Mat map = p.getMap();
        int num = 0;
        for(auto entry = map.begin<cv::Vec3f>(); entry != map.end<cv::Vec3f>(); ++entry){
            if(*entry != cv::Vec3f{0,0,0}){
                ++num;
            }
        }
        double ratio_used = double(num) /(width * height );
        double ratio_points = num / double(n);
        double q = ratio_points * ratio_used;
        if(q > opt) {
            opt = q;
            best = p;
        }
        p.clear();
        std::cout << "Panorama size " << width << "x" << height << " used pixels " << ratio_used
            << " used points " << num  << '\n'; //<< " measure " << q << '\n';
#ifndef NDEBUG
        cv::imwrite("debug/panoramas/"+std::to_string(width)+"x"+std::to_string(height)+".png", p.getNormalizedRangeImage());
#endif
    }
#ifndef NDEBUG
    std::cout << "Optimum " << opt << '\n';
#endif
    return best;
}

std::vector<segment> post_process(std::vector<plane_candidate>& planes, double cell_size){
    std::vector<segment> segments;
    segments.push_back({{}, planes[0].normal, planes[0].distance});
    for(auto p : planes[0].points) segments[0].points.emplace_back(*p);
    for(auto it = planes.begin() + 1; it != planes.end(); ++it) {
#ifndef NDEBUG
        std::cout << "prost processing plane " << (it-planes.begin()) << "/" << planes.size()-1;
#endif
        std::vector<segment> plane_segments = segment_plane(*it, cell_size);
#ifndef NDEBUG
        std::cout << " - found " << plane_segments.size() << " segments" << std::endl;
        write_scan(plane_segments, "debug/post_processing/plane"+std::to_string(it-planes.begin()), 0,
            [](const segment& s){return "n:"+cvVecnT_string(s.normal)+" d:"+std::to_string(s.distance);}, identity(),
            [](const segment& s){return s.points;});
#endif
        segments.insert(segments.cend(), plane_segments.begin(), plane_segments.end());
    }
    return segments;
}

std::vector<segment> segment_scan(scan_cv& scan, size_t panorama_width, size_t panorama_height, size_t window_size,
                                  size_t minimum_points, double tau, double cell_size, bool disable_post_processing)
{
#ifndef NDEBUG
    //std::system("rm -rf debug");
    mkdir("debug", S_IRWXU|S_IRWXG|S_IRWXO);
    //mkdir("debug/panoramas", S_IRWXU|S_IRWXG|S_IRWXO);
    //mkdir("debug/normals", S_IRWXU|S_IRWXG|S_IRWXO);
    mkdir("debug/distances", S_IRWXU|S_IRWXG|S_IRWXO);
    mkdir("debug/components", S_IRWXU|S_IRWXG|S_IRWXO);
    mkdir("debug/grids",  S_IRWXU|S_IRWXG|S_IRWXO);
    mkdir("debug/post_processing",  S_IRWXU|S_IRWXG|S_IRWXO);
#endif
    
    std::cout << "Projecting scan" << std::endl;
    using fbr::panorama;
    //panorama image;
    panorama image = best_size_panorama(scan, panorama_width, panorama_height);
#ifndef NEDBUG
    cv::imwrite("debug/panorama.png",image.getNormalizedRangeImage());
#endif
    
    cv::Mat map = image.getMap();
    std::cout << "Calculating distance measure" << std::endl;
    std::vector<std::vector<segment>> phis(map.rows, std::vector<segment>(map.cols));
    similarity_measure(map, window_size, phis);
    
    std::cout << "Building Graph" << std::endl;
    using namespace david_graph;
    grid_graph<double> graph = make_grid_graph<double>(map.rows, map.cols, 0.0);
    /*remove zero vertices*/
    graph.remove_vertices([&](const std::pair<int, int>& v){return map.at<cv::Vec3f>(v.first, v.second) == cv::Vec3f{0,0,0};});
    edge_distances(image.getMap(), phis, graph);
    
    std::cout << "Cutting graph" << std::endl;
    std::vector<plane_candidate> planes = cut_graph(graph, map, minimum_points, tau);
#ifndef NDEBUG
    std::ofstream plane_list("debug/planes.txt");
    for(size_t i = 0; i < planes.size(); ++i) {
        plane_list << i << ": " << planes[i].normal << ' ' << planes[i].distance << '\n';
    }
    plane_list.close();
#endif
    /** DONT RETURN POINTERS TO THE POINTS (E.G. POINTERS IN PLANE_CANDIDATES) FROM THIS FUNCTION SINCE THEY POINT INTO
     * THE PANORAMA*/
    /*BEcause we cluster planes and not segments, post_processing cuts the planes into segments again*/
    if(!disable_post_processing) {
        std::cout << "Found " << planes.size() - 1 << " planes\nPost processing" << std::endl; 
        std::vector<segment> segments =  post_process(planes, cell_size);
        std::cout << "Found " << segments.size() - 1 << " segments" << std::endl;
        return segments;
    } else {
        std::cout << "Found " << planes.size() - 1<< " segments" << std::endl;
        std::vector<segment> s(planes.size());
        for(size_t i = 0; i < s.size(); ++i) {
            for(const cv::Vec3f* p : planes[i].points){
                s[i].points.emplace_back(*p);
            };
            s[i].normal = planes[i].normal;
            s[i].distance = planes[i].distance;
        }
        return s;
    }
}


