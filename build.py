'''
Created on Apr 9, 2018

@author: modys
'''


###################################################### Includes ########################################################
import os
import shutil
import platform
import subprocess
import argparse
########################################################################################################################


################################################## Common Functions ####################################################

#----------------------------------------------------------------------------------------------------------------------#
'''
/**
 * @brief _clean ,Helper function to erase build folders.
 *
 * @param[in]  dest_path
 *  The destination path to the build folder.
 *
 * @param[in]  dest_folder
 *  The build folder name.
 *
 *  @return  None.
 */
'''
def _clean (dest_path,dest_folder):
    #construct build folder path
    if(dest_path == './'):
        folder_full_path = dest_folder
    else:
        folder_full_path = dest_path+'/'+dest_folder

    print("---------Clean------------")

    ## Try to remove tree; if failed show an error using try...except on screen
    try:
        shutil.rmtree(folder_full_path)
    except OSError as e:
        print ("Error: %s - %s." % (e.filename,e.strerror))
#----------------------------------------------------------------------------------------------------------------------#

#----------------------------------------------------------------------------------------------------------------------#    
'''
/**
 * @brief _build ,Helper function to build using cmake.
 *
 * @param[in]  cmake_flag
 *  The cmake flag is used in cmake to generate the correct makefile.
 *
 * @param[in]  dest_path
 *  The destination path to the build folder.
 *
 * @param[in]  dest_folder
 *  The build folder name.
 *
 *  @return  None.
 */
'''
def _build (cmake_flag,dest_path,dest_folder):
    #construct build folder path
    if(dest_path == './'):
        folder_full_path = dest_folder
    else:
        folder_full_path = dest_path+'/'+dest_folder

    if(cmake_flag is not ''):
        cmake_flag ='-D'+ cmake_flag + '=1'
    

    # Create a new directory
    if not os.path.exists(folder_full_path):
        print("---------Create New Build Directory------------")
        print(folder_full_path)
        os.makedirs(folder_full_path)
        
    print("---------Build------------")
      
    if(platform.system() == 'Windows'):
        print("---------Build Windows(Not Yet Done)------------")
        print("%s is ON"%(cmake_flag))
        cmd = 'cd'+' '+folder_full_path
        cmd =  cmd + ' & cmake '+cmake_flag+' cmake .. -G "MinGW Makefiles" & mingw32-make'
        print("cmd = %s"%cmd)
        subprocess.call(cmd,shell=True)
            
    elif (platform.system()=='Linux'):
        print("---------Build Linux--------------")
        print("%s is ON"%(cmake_flag))
        cmd = 'cd'+' '+os.path.abspath(folder_full_path)
        cmd = cmd + '&& cmake '+cmake_flag+' cmake .. && make'
        print("cmd = %s"%cmd)
        subprocess.call(cmd,shell=True)
            
    else:
        print("Platform Unknown")
#----------------------------------------------------------------------------------------------------------------------#

#----------------------------------------------------------------------------------------------------------------------#
'''
/**
 * @brief _run ,Helper function to rrun the output and build the ocde if needed.
 *
 * @param[in]  cmake_flag
 *  The cmake flag is used in cmake to generate the correct makefile.
 *
 * @param[in]  dest_path
 *  The destination path to the build folder.
 *
 * @param[in]  dest_folder
 *  The build folder name.
 *
 * @param[in]  app_name
 *  The output of build process.
 *
 *  @return  None.
 */
'''
def _run(cmake_flag,dest_path,dest_folder,app_name,extraParam=None):
    _build(cmake_flag,dest_path,dest_folder)

    if(extraParam is None):
        extraParam = ''

    print("---------Run Application------------")
    #construct build folder path
    if(dest_path == './'):
        folder_full_path = dest_folder
    else:
        folder_full_path = dest_path+'/'+dest_folder

    if(platform.system() == 'Windows'):
        print("---------Run App on Windows(Not Yet Done)------------")
        cmd = 'cd'+' '+folder_full_path+ ' '+app_name+'.exe'
        print("cmd = %s"%cmd)
        subprocess.call(cmd,shell=True)
            
    elif (platform.system()=='Linux'):
        print("---------Run App on Linux--------------")
        cmd = 'cd'+' '+os.path.abspath(folder_full_path)+' && ./'+app_name+' '+extraParam
        print("cmd = %s"%cmd)
        subprocess.call(cmd,shell=True)
            
    else:
        print("Platform Unknown")
#----------------------------------------------------------------------------------------------------------------------#


######################################################## Main ##########################################################

if __name__ == '__main__':
    pass

#------------------------------------------------ Utility Options -----------------------------------------------------#
print("--------Project Build System--------------")
print("clean --> Cleans the project")
print("build --> builds the project according to platform type")
print("run --> runs the project")
print("ray_mode --> Use Ray Mode")
print("point_cloud_mode --> Use Point Cloud Mode")
print("seg_mode --> Use segmentation Mode")
print("ransac_2d_mode --> Use Ransac 2D Mode")
print("ransac_3d_mode --> Use Ransac 3D Mode")
print("pcl_cluster_mode --> Use Pcl Cluster Mode")
print("self_cluster_mode --> Use Self Cluster Mode")
print("self_cluster_ransc_mode --> Use Self Cluster/Ransac Mode")
print("pcl_filter_mode --> Use Pcl Filter Mode")
print("final_project_mode --> Use Final Project Mode")
print("final_project_self_mode --> Use Final Project Mode(RANAC_ECULDIAN+CLUSTRING)")


parser = argparse.ArgumentParser()
parser.add_argument("--clean", help="Cleans the project",action="store_true")
parser.add_argument("--build", help="builds the project according to platform type",action="store_true")
parser.add_argument("--run", help="runs the project",action="store_true")
parser.add_argument("--ray_mode", help="Use Ray Mode",action="store_true")
parser.add_argument("--point_cloud_mode", help="Use Point Cloud Mode",action="store_true")
parser.add_argument("--seg_mode", help="Use segmentation Mode",action="store_true")
parser.add_argument("--ransac_2d_mode", help="Use Ransac 2D Mode",action="store_true")
parser.add_argument("--ransac_3d_mode", help="Use Ransac 3D Mode",action="store_true")
parser.add_argument("--pcl_cluster_mode", help="Use Pcl Cluster Mode",action="store_true")
parser.add_argument("--self_cluster_mode", help="Use Self Cluster Mode",action="store_true")
parser.add_argument("--self_cluster_ransc_mode", help="Use Self Cluster/Ransac Mode",action="store_true")
parser.add_argument("--pcl_filter_mode", help="Use Pcl Filter Mode",action="store_true")
parser.add_argument("--final_project_mode", help="Use Final Project Mode",action="store_true")
parser.add_argument("--final_project_self_mode", help="Use Final Project Mode",action="store_true")
parser.add_argument("--data1", help="Use data set 1",action="store_true")
parser.add_argument("--data2", help="Use data set 2",action="store_true")
args = parser.parse_args()
#----------------------------------------------------------------------------------------------------------------------#

#-------------------------------------------------- Temp Variables ----------------------------------------------------#
folder_path = './'
folder      = 'build'
app_name    = 'environment'
cmake_flag  = ''
data_set    = ''      
extraParam  = None
data_set    = None
#----------------------------------------------------------------------------------------------------------------------#


#-------------------------------------------------- States Machine ----------------------------------------------------#
#-------------------------------------------------- Ray Parameter -----------------------------------------------------#
if args.ray_mode:
    folder_path = './'
    folder      = 'build'
    app_name    = 'environment'
    cmake_flag  = 'USE_RAY'

#---------------------------------------------- Point Cloud Parameter -------------------------------------------------#
if args.point_cloud_mode:
    folder_path = './'
    folder      = 'build'
    app_name    = 'environment'
    cmake_flag  = 'USE_POINT_CLOUD'

#--------------------------------------------- Segmentation Parameter -------------------------------------------------#
if args.seg_mode:
    folder_path = './'
    folder      = 'build'
    app_name    = 'environment'
    cmake_flag  = 'USE_SEG'

#--------------------------------------------------- RANSAC 2D --------------------------------------------------------#
if args.ransac_2d_mode:
    folder_path = './src/quiz/ransac'
    folder      = 'build'
    app_name    = 'quizRansac'
    cmake_flag  = 'USE_RANSAC_2D'

#--------------------------------------------------- RANSAC 3D --------------------------------------------------------#
if args.ransac_3d_mode:
    folder_path = './src/quiz/ransac'
    folder      = 'build'
    app_name    = 'quizRansac'
    cmake_flag  = 'USE_RANSAC_3D'

#-------------------------------------------------- pcl_cluster -------------------------------------------------------#
if args.pcl_cluster_mode:
    folder_path = './'
    folder      = 'build'
    app_name    = 'environment'
    cmake_flag  = 'USE_PCL_CLUSTER'

#------------------------------------------------- slef_cluster -------------------------------------------------------#
if args.self_cluster_mode:
    folder_path = './src/quiz/cluster'
    folder      = 'build'
    app_name    = 'quizCluster'
    cmake_flag  = 'USE_SELF_CLUSTER'

#-------------------------------------------------- pcl_filter --------------------------------------------------------#
if args.pcl_filter_mode:
    folder_path = './'
    folder      = 'build'
    app_name    = 'environment'
    cmake_flag  = 'USE_PCL_FILTER'

#--------------------------------------------- slef_cluster_ransac ----------------------------------------------------#
if args.self_cluster_ransc_mode:
    folder_path = './src/quiz/cluster_ransac'
    folder      = 'build'
    app_name    = 'quizCluster'
    cmake_flag  = 'USE_SELF_CLUSTER_RANSAC'
    extraParam  

#--------------------------------------------------- Data Set  --------------------------------------------------------#
if args.data1:
    data_set = 'data_1'

if args.data2:
    data_set = 'data_2'

#------------------------------------------------ Final Project  ------------------------------------------------------#
if args.final_project_mode:
    if data_set is None:
        data_set = 'data_1'

    folder_path           = './'
    folder                = 'build'
    app_name              = 'environment'
    cmake_flag            = 'USE_FINAL_PROJECT'
    data_path             = '../src/sensors/data/pcd/'
    filterRes             = '0.15'
    filterMinPoint        = '-10.0,-5.0,-2.0'
    filtermaxPoint        = '25.0,7.0,2.0'
    segDistanceThreshold  = '0.2'
    segMaxIterations      = '50'
    clusterTolerance      = '0.2'
    clusterMinSize        = '10'
    clusterMaxSize        = '1000'
    extraParam            = data_path+data_set+','+filterRes+','+\
                            filterMinPoint+','+filtermaxPoint+','+\
                            segDistanceThreshold+','+segMaxIterations+','+\
                            clusterTolerance+','+clusterMinSize+','+clusterMaxSize

#--------------------------------------------- Final Project Self  ----------------------------------------------------#
if args.final_project_self_mode:
    if data_set is None:
        data_set = 'data_1'
    folder_path           = './'
    folder                = 'build'
    app_name              = 'environment'
    cmake_flag            = 'USE_FINAL_PROJECT_SELF'
    data_path             = '../src/sensors/data/pcd/'
    filterRes             = '0.15'
    filterMinPoint        = '-20.0,-6.0,-2.0'
    filtermaxPoint        = '30.0,7.0,5.0'
    segDistanceThreshold  = '0.2'
    segMaxIterations      = '50'
    clusterTolerance      = '0.3'
    clusterMinSize        = '10'
    clusterMaxSize        = '800'
    extraParam            = data_path+data_set+','+filterRes+','+\
                            filterMinPoint+','+filtermaxPoint+','+\
                            segDistanceThreshold+','+segMaxIterations+','+\
                            clusterTolerance+','+clusterMinSize+','+clusterMaxSize

print(extraParam)
if args.clean:
    _clean(folder_path, folder)
 
if args.build:
    _build(cmake_flag,folder_path,folder)
    
if args.run:
    _run(cmake_flag, folder_path,folder,app_name,extraParam)

#----------------------------------------------------------------------------------------------------------------------#


########################################################################################################################