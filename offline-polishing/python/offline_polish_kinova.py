#!/usr/bin/env python
from __future__ import division
# works with python3 ubuntu 20
# in the terminal do "source catkin/devel/setup.bash to add the kortex_driver to the directory

# THIS IS ONE EXAMPLE AND THE RADIUS WAS CUT BY /3 BECAUSE THE POLISHING MOTION IS TOO BIG
from scipy.optimize import minimize
from scipy.optimize import Bounds
from kortex_driver.msg import Base_JointSpeeds
from kortex_driver.msg import TwistCommand
from kortex_driver.msg import Twist
from geometry_msgs.msg import PoseStamped
import rospy
import tf
import os
import numpy as np
import math
import sys
import signal
import matplotlib.pyplot as plt

FREQ = 40
dt = 1/120
last_cup = ''
active_cup = False
once_1 = True

points = []
point_x = []
point_y = []


def fun(x):
    x_hat = x[3] * math.cos(x[5]) * (Xdata[0]) + x[3] * math.sin(x[5]) * (Xdata[1])
    y_hat = -x[4] * math.sin(x[5]) * (Xdata[0]) + x[4] * math.cos(x[5]) * (Xdata[1])

    r_ = np.sqrt(np.power(x_hat, 2) + np.power(y_hat, 2))
    phi = np.arctan2(y_hat, x_hat)

    r__dot = -1 * x[0] * (r_ - x[2])
    phi_dot = x[1]

    xd_hat = r__dot * np.cos(phi) - r_ * phi_dot * np.sin(phi)
    yd_hat = r__dot * np.sin(phi) + r_ * phi_dot * np.cos(phi)

    xdot_d1 = np.cos(x[5]) * (1 / x[3]) * xd_hat - np.sin(x[5]) * (1 / x[4]) * yd_hat
    xdot_d2 = np.sin(x[5]) * (1 / x[3]) * xd_hat + np.cos(x[5]) * (1 / x[4]) * yd_hat

    return np.sum(np.power((r_ - r) / np.linalg.norm(r, ord=1), 2)) + np.sum(
        np.power((phi - theta_circle) / np.linalg.norm(theta_circle, ord=1), 2)) + np.sum(
        np.power((r__dot - r_dot) / np.linalg.norm(r_dot, ord=1), 2)) + np.sum(
        np.power((phi_dot - theta_circle_dot) / np.linalg.norm(theta_circle_dot, ord=1), 2))


class Mocap:

    def __init__(self):
        self.init_params()
        self.init_ros()

        print('Starting node..')

        self.get_robot_pos()
        ## tilt down
        leave = False
        while not leave:
            # go to initial pose
            # start = np.array([0.35, -0.45, 0])
            start = np.array([0.55, 0, 0.2])

            self.set_pose_down(start)
            self.get_robot_pos()
            if self.ee_real_logged:
                vel_cmd = self.calculate_robot_desired_orientation()
                self.publish_robot_desired(vel_cmd)
                print("1- Difference between desired orientation and actual\n" + str(np.round(
                    np.linalg.norm(self.desired_orientation - self.quat_to_euler(self.end_efector_orientation)),
                    2)))
                if (np.round(
                        np.linalg.norm(self.desired_orientation - self.quat_to_euler(self.end_efector_orientation)),
                        2)) < 0.1:
                    leave = True
            signal.signal(signal.SIGINT, self.signal_handler)

        # stop robot
        vel_cmd = [0, 0, 0, 0, 0, 0]
        self.publish_robot_desired(vel_cmd)
        rospy.sleep(1)

        # get data of polishing motion
        close = False
        while not close:
            global r, theta_circle, r_dot, theta_circle_dot, Xdata

            # 1 example
            data = [[0] * 200] * 2
            data[0] = np.array([0.215537022658653, 0.230061586860520, 0.243835403145969, 0.257028047742788,
                                0.269746995461700, 0.282058342494254, 0.294000919362920, 0.305595784318122,
                                0.316852540109286, 0.327773498033122, 0.338356401387847, 0.348596195153218,
                                0.358486169916400, 0.368018698411902, 0.377185708580767, 0.385978987172165,
                                0.394390374869278, 0.402411892240650, 0.410035821704965, 0.417254761572808,
                                0.424061662364878, 0.430449851856876, 0.436413052915237, 0.441945396675662,
                                0.447041432661522, 0.451696136838359, 0.455904918223803, 0.459663624436547,
                                0.462968546421041, 0.465816422493261, 0.468204441796274, 0.470130247219447,
                                0.471591937813662, 0.472588070721871, 0.473117662636378, 0.473180190789624,
                                0.472775593482456, 0.471904270152461, 0.470567080984145, 0.468765346062483,
                                0.466500844071387, 0.463775810538781, 0.460592935630288, 0.456955361493847,
                                0.452866679158003, 0.448330924986953, 0.443352576695913, 0.437936548930754,
                                0.432088188416263, 0.425813268677857, 0.419117984341947, 0.412008945020581,
                                0.404493168786411, 0.396578075244429, 0.388271478207299, 0.379581577981535,
                                0.370516953272123, 0.361086552713588, 0.351299686035866, 0.341166014873703,
                                0.330695543228648, 0.319898607593059, 0.308785866745881, 0.297368291230252,
                                0.285657152523342, 0.273664011909098, 0.261400709064904, 0.248879350373400,
                                0.236112296971012, 0.223112152544988, 0.209891750890986, 0.196464143243499,
                                0.182842585391622, 0.169040524592884, 0.155071586298062, 0.140949560700076,
                                0.126688389120258, 0.112302150245421, 0.0978050462293220,
                                0.0832113886722397, 0.0685355844925010, 0.0537921217039148,
                                0.0389955551131447, 0.0241604919511447, 0.00930157745284123,
                                -0.00556651960070279, -0.0204291213633684, -0.0352715554140103,
                                -0.0500791692411308, -0.0648373447078989, -0.0795315124832351,
                                -0.0941471664247139, -0.108669877899087, -0.123085310026291,
                                -0.137379231832868, -0.151537532300850, -0.165546234298203,
                                -0.179391508377118, -0.193059686426492, -0.206537275165147,
                                -0.219810969462450, -0.232867665473193, -0.245694473573762,
                                -0.258278731086829, -0.270608014781992, -0.282670153140047,
                                -0.294453238368756, -0.305945638158271, -0.317136007164593,
                                -0.328013298209750, -0.338566773187608, -0.348786013664578,
                                -0.358660931164737, -0.368181777129206, -0.377339152539976,
                                -0.386124017198651, -0.394527698650971, -0.402541900748293,
                                -0.410158711837583, -0.417370612571835, -0.424170483333196,
                                -0.430551611261487, -0.436507696881162, -0.442032860320179,
                                -0.447121647114629, -0.451769033593413, -0.455970431837623,
                                -0.459721694209756, -0.463019117448283, -0.465859446323519,
                                -0.468239876851201, -0.470158059060593, -0.471612099314385,
                                -0.472600562178102, -0.473122471837169, -0.473177313060239,
                                -0.472765031707833, -0.471886034785785, -0.470541190043447,
                                -0.468731825117039, -0.466459726219003, -0.463727136374649,
                                -0.460536753207830, -0.456891726277833, -0.452795653970127,
                                -0.448252579944019, -0.443266989140738, -0.437843803355880,
                                -0.431988376380596, -0.425706488716304, -0.419004341868155,
                                -0.411888552222885, -0.404366144517084, -0.396444544902355,
                                -0.388131573614182, -0.379435437251763, -0.370364720676422,
                                -0.360928378536600, -0.351135726427784, -0.340996431696123,
                                -0.330520503894773, -0.319718284902436, -0.308600438713806,
                                -0.297177940912037, -0.285462067833596, -0.273464385436225,
                                -0.261196737880973, -0.248671235839596, -0.235900244538853,
                                -0.222896371553500, -0.209672454360045, -0.196241547663536,
                                -0.182616910509902, -0.168811993196573, -0.154840423994286,
                                -0.140715995693206, -0.126452651986622, -0.112064473705680,
                                -0.0975656649187275, -0.0829705389089981, -0.0682935040444820,
                                -0.0535490495539250, -0.0387517312230032, -0.0239161570247914,
                                -0.00905697269871303, 0.00581115270779389, 0.0206735413206202,
                                0.0355155209290431, 0.0503224394701919, 0.0650796794936249,
                                0.0797726725917361, 0.0943869137817461, 0.108907975825081,
                                0.123321523470001, 0.137613327603419, 0.151769279297941, 0.165775403740252,
                                0.179617874027116, 0.193283024815346, 0.206757365812287])
            data[1] = np.array(
                [-0.397587824387214, -0.392463602571039, -0.385578363660813, -0.377500159133904,
                 -0.368569520654282, -0.358992056213825, -0.348893521260631, -0.338352526071555,
                 -0.327419934171049, -0.316130350912825, -0.304508916982266, -0.292575318444997,
                 -0.280346148645289, -0.267836295377197, -0.255059752292022, -0.242030090614242,
                 -0.228760730679144, -0.215265095636938, -0.201556695863069, -0.187649172650268,
                 -0.173556317982977, -0.159292080259527, -0.144870561748281, -0.130306011168342,
                 -0.115612813380619, -0.100805477352652, -0.0858986230799390, -0.0709069678661558,
                 -0.0558453122013930, -0.0407285253827186, -0.0255715309663919, -0.0103892921092727,
                 0.00480320316136012, 0.0199909567209121, 0.0351589751408752, 0.0502922844833721,
                 0.0653759450796683, 0.0803950662766238, 0.0953348211357225, 0.110180461069725,
                 0.124917330402251, 0.139530880835784, 0.154006685813751, 0.168330454762446,
                 0.182488047198740, 0.196465486689620, 0.210248974649782, 0.223824903963658,
                 0.237179872418413, 0.250300695934670, 0.263174421581888, 0.275788340365548,
                 0.288129999773526, 0.300187216069265, 0.311948086319608, 0.323401000145425,
                 0.334534651183424, 0.345338048247841, 0.355800526180983, 0.365911756381915,
                 0.375661757002898, 0.385040902803510, 0.394039934652719, 0.402649968669540,
                 0.410862504993238, 0.418669436174419, 0.426063055178747, 0.433036062995351,
                 0.439581575842443, 0.445693131963014, 0.451364698003909, 0.456590674971975,
                 0.461365903761410, 0.465685670246856, 0.469545709937200, 0.472942212185499,
                 0.475871823950866, 0.478331653108604, 0.480319271305325, 0.481832716356229,
                 0.482870494182182, 0.483431580284678, 0.483515420757225, 0.483121932832166,
                 0.482251504962387, 0.480904996437836, 0.479083736537226, 0.476789523215768,
                 0.474024621330218, 0.470791760403009, 0.467094131927648, 0.462935386218063,
                 0.458319628804999, 0.453251416383022, 0.447735752312127, 0.441778081678407,
                 0.435384285918636, 0.428560677014096, 0.421313991259359, 0.413651382612189,
                 0.405580415631131, 0.397109058007744, 0.388245672700861, 0.378999009680648,
                 0.369378197290593, 0.359392733235971, 0.349052475207668, 0.338367631150628,
                 0.327348749186533, 0.316006707200649, 0.304352702103134, 0.292398238775406,
                 0.280155118712471, 0.267635428372447, 0.254851527244755, 0.241816035648787,
                 0.228541822275071, 0.215041991481250, 0.201329870355403, 0.187418995559495,
                 0.173323099965924, 0.159056099100373, 0.144632077404350, 0.130065274330969,
                 0.115370070287701, 0.100560972439980, 0.0856526003896665, 0.0706596717425216,
                 0.0555969875789232, 0.0404794178421797, 0.0253218866588615, 0.0101393576056422,
                 -0.00505318106280517, -0.0202407312102748, -0.0354082996252490, -0.0505409128222581,
                 -0.0656236318237663, -0.0806415669079912, -0.0955798923080982, -0.110423860848258,
                 -0.125158818502118, -0.139770218859319, -0.154243637485769, -0.168564786163503,
                 -0.182719526996072, -0.196693886365531, -0.210474068727252, -0.224046470228943,
                 -0.237397692140423, -0.250514554080907, -0.263384107030730, -0.275993646114673,
                 -0.288330723144266, -0.300383158906693, -0.312139055188157, -0.323586806519849,
                 -0.334715111634913, -0.345512984625101, -0.355969765786108, -0.366075132140876,
                 -0.375819107630480, -0.385192072962529, -0.394184775107378, -0.402788336432752,
                 -0.410994263467780, -0.418794455287789, -0.426181211511564, -0.433147239903201,
                 -0.439685663571030, -0.445790027756507, -0.451454306206385, -0.456672907121849,
                 -0.461440678678769, -0.465752914113597, -0.469605356369902, -0.472994202300962,
                 -0.475916106424234, -0.478368184224039, -0.480348014999160, -0.481853644252573,
                 -0.482883585620930, -0.483436822341908, -0.483512808257957, -0.483111468355470,
                 -0.482233198838837, -0.480878866739312, -0.479049809059073, -0.476747831451330,
                 -0.473975206437777, -0.470734671165147, -0.467029424703094, -0.462863124886054,
                 -0.458239884702216, -0.453164268233169, -0.447641286148214, -0.441676390757816,
                 -0.435275470631061, -0.428444844782432, -0.421191256433657, -0.413521866356771])

            # 2nd example (you need to change the size of the Xdata to 100)
            # data = [[0]*100]*2
            # data[0] = np.array([0.500000000000000,0.499013364214136,0.496057350657239,0.491143625364344,0.484291580564316,0.475528258147577,0.464888242944126,0.452413526233010,0.438153340021932,0.422163962751008,0.404508497187474,0.385256621387895,0.364484313710706,0.342273552964344,0.318711994874345,0.293892626146237,0.267913397489498,0.240876837050858,0.212889645782536,0.184062276342339,0.154508497187474,0.124344943582427,0.0936906572928623,0.0626666167821521,0.0313952597646567,-8.04061324838318e-17,-0.0313952597646567,-0.0626666167821522,-0.0936906572928624,-0.124344943582427,-0.154508497187474,-0.184062276342339,-0.212889645782536,-0.240876837050858,-0.267913397489498,-0.293892626146237,-0.318711994874345,-0.342273552964344,-0.364484313710706,-0.385256621387895,-0.404508497187474,-0.422163962751008,-0.438153340021932,-0.452413526233010,-0.464888242944126,-0.475528258147577,-0.484291580564316,-0.491143625364344,-0.496057350657239,-0.499013364214136,-0.500000000000000,-0.499013364214136,-0.496057350657239,-0.491143625364344,-0.484291580564316,-0.475528258147577,-0.464888242944126,-0.452413526233010,-0.438153340021932,-0.422163962751008,-0.404508497187474,-0.385256621387895,-0.364484313710706,-0.342273552964345,-0.318711994874345,-0.293892626146237,-0.267913397489499,-0.240876837050858,-0.212889645782536,-0.184062276342339,-0.154508497187474,-0.124344943582428,-0.0936906572928627,-0.0626666167821523,-0.0313952597646566,-9.18485099360515e-17,0.0313952597646564,0.0626666167821521,0.0936906572928621,0.124344943582427,0.154508497187474,0.184062276342339,0.212889645782536,0.240876837050857,0.267913397489498,0.293892626146236,0.318711994874345,0.342273552964344,0.364484313710706,0.385256621387895,0.404508497187474,0.422163962751007,0.438153340021932,0.452413526233010,0.464888242944126,0.475528258147577,0.484291580564316,0.491143625364344,0.496057350657239,0.499013364214136,0.500000000000000])
            # data[1] = np.array([0,0.0941857792939701,0.187999850346456,0.281071971878587,0.373034830747282,0.463525491562421,0.552186829027017,0.638668937347609,0.722630511152573,0.803740192468495,0.881677878438710,0.956135984623035,1.02682065889303,1.09345294113212,1.15576986416368,1.21352549156242,1.26649188825302,1.31446002006580,1.35724057869903,1.39466472883238,1.42658477444273,1.45287474169295,1.47343087609303,1.48817205197172,1.49704009264241,1.50000000000000,1.49704009264241,1.48817205197172,1.47343087609303,1.45287474169295,1.42658477444273,1.39466472883238,1.35724057869903,1.31446002006580,1.26649188825302,1.21352549156242,1.15576986416368,1.09345294113212,1.02682065889303,0.956135984623034,0.881677878438710,0.803740192468495,0.722630511152573,0.638668937347609,0.552186829027017,0.463525491562421,0.373034830747282,0.281071971878587,0.187999850346456,0.0941857792939697,1.83697019872103e-16,-0.0941857792939694,-0.187999850346456,-0.281071971878586,-0.373034830747282,-0.463525491562420,-0.552186829027016,-0.638668937347608,-0.722630511152573,-0.803740192468495,-0.881677878438710,-0.956135984623034,-1.02682065889303,-1.09345294113212,-1.15576986416368,-1.21352549156242,-1.26649188825302,-1.31446002006579,-1.35724057869903,-1.39466472883238,-1.42658477444273,-1.45287474169295,-1.47343087609303,-1.48817205197172,-1.49704009264241,-1.50000000000000,-1.49704009264241,-1.48817205197172,-1.47343087609303,-1.45287474169295,-1.42658477444273,-1.39466472883238,-1.35724057869903,-1.31446002006580,-1.26649188825302,-1.21352549156242,-1.15576986416368,-1.09345294113212,-1.02682065889303,-0.956135984623034,-0.881677878438710,-0.803740192468496,-0.722630511152573,-0.638668937347610,-0.552186829027017,-0.463525491562421,-0.373034830747283,-0.281071971878587,-0.187999850346457,-0.0941857792939699,-3.67394039744206e-16])

            dt = 1 / 50
            # remove last one
            Xdata = [sub[:199] for sub in data]

            # Xdata = np.array(data)
            Xvel = np.diff(data) / dt

            # convert to polar coordinates
            r = np.sqrt(np.power(Xdata[0], 2) + np.power(Xdata[1], 2))
            theta_circle = np.arctan2(Xdata[1], Xdata[0])

            # convert to polar velocity coordinates
            r_dot = np.sum(Xdata * Xvel, axis=0) / np.sqrt(
                np.power(Xdata[0], 2) + np.power(Xdata[1], 2))  # param 1
            theta_circle_dot = (Xvel[1] * Xdata[0] - Xdata[1] * Xvel[0]) / (
                        np.power(Xdata[0], 2) + np.power(Xdata[1], 2))  # param 2

            # Xdata = [sub[:799] for sub in points]
            # # print('0', Xdata[0])
            # # print('1', Xdata[1])
            #
            # # Xdata = np.array(data)
            # Xvel = np.diff(points) / dt
            #
            # # convert to polar coordinates
            # r = np.sqrt(np.power(Xdata[0], 2) + np.power(Xdata[1], 2))
            # theta_circle = np.arctan2(Xdata[1], Xdata[0])
            #
            # # convert to polar velocity coordinates
            # r_dot = np.sum(Xdata * Xvel, axis=0) / np.sqrt(
            #     np.power(Xdata[0], 2) + np.power(Xdata[1], 2))  # param 1
            # theta_circle_dot = (Xvel[1] * Xdata[0] - Xdata[1] * Xvel[0]) / (
            #             np.power(Xdata[0], 2) + np.power(Xdata[1], 2))  # param 2

            # initial parameters
            #  alpha omega radius a b theta
            x0 = [1, 1, 0.1, 0.1, 0.1, 0.1]
            # bounds
            bnds = Bounds([5, -2 * math.pi, 0.0001, 1, 1, -2 * math.pi],
                          [50, 2 * math.pi, 2, np.infty, np.infty, 2 * math.pi])

            res_robust = minimize(fun, x0, method='SLSQP', bounds=bnds)
            print(res_robust.x)
            popt = res_robust.x
            self.popt = popt

            # # Plotting Results
            w = 1
            Y, X = np.mgrid[-w:w:0.1, -w:w:0.1]

            # compute dynamics
            x_hat = popt[3] * math.cos(popt[5]) * X + popt[3] * math.sin(popt[5]) * Y
            y_hat = -popt[4] * math.sin(popt[5]) * X + popt[4] * math.cos(popt[5]) * Y

            r_ = np.sqrt(np.power(x_hat, 2) + np.power(y_hat, 2))
            phi = np.arctan2(y_hat, x_hat)

            r__dot = -1 * np.multiply(popt[0], (r_ - popt[2]))
            phi_dot = popt[1]

            xd_hat = r__dot * np.cos(phi) - r_ * phi_dot * np.sin(phi)
            yd_hat = r__dot * np.sin(phi) + r_ * phi_dot * np.cos(phi)

            U = math.cos(popt[5]) * (1 / popt[3]) * xd_hat - math.sin(popt[5]) * (1 / popt[4]) * yd_hat
            V = math.sin(popt[5]) * (1 / popt[3]) * xd_hat + math.cos(popt[5]) * (1 / popt[4]) * yd_hat

            # plotting the data
            fig = plt.figure(1)
            ax0 = fig.subplots()
            l1 = ax0.streamplot(X, Y, U, V, density=[2, 2])
            ax0.plot(data[0], data[1], '.r', label='real trajectory')
            plt.xlabel('$x$')
            plt.ylabel('$y$')
            ax0.set_title('Limit Cycle')
            ax0.legend()
            plt.tight_layout()
            plt.savefig('output.png')
            os.system("xv output.png &")

            close = True

            signal.signal(signal.SIGINT, self.signal_handler)

        # # Robot Polish
        leave = False
        # start = time.time()
        # do a limit cycle
        while not leave:
            self.set_pose([])
            self.get_robot_pos()
            self.circle_center = [0.5, -0.1, 0.03]   # Change the center specially the height to prevent crashing
            if self.ee_real_logged:
                # param 3 (r_value)
                self.circle_vel = self.ellipse_ds(self.end_effector_position - self.circle_center, self.popt[2]/3)
                self.publish_robot_desired(self.circle_vel)
                self.rate.sleep()

            signal.signal(signal.SIGINT, self.signal_handler)

        # stop robot
        vel_cmd = [0, 0, 0, 0, 0, 0]
        self.publish_robot_desired(vel_cmd)
        rospy.sleep(1)

    def ellipse_ds(self, x, r_value):
        a = self.popt[3]  # 1 param 4
        b = self.popt[4]  # 3 param 5
        theta = self.popt[5]  # math.pi / 3  # param 6

        x_hat = a * math.cos(theta) * (x[0]) + a * math.sin(theta) * (x[1])
        y_hat = -b * math.sin(theta) * (x[0]) + b * math.cos(theta) * (x[1])

        # parameters for limit cycle
        theta_circle = math.atan2(y_hat, x_hat)
        r = np.sqrt(x_hat ** 2 + y_hat ** 2)

        theta_circle_dot = self.popt[1]  # math.pi / 3  # param 1
        r_dot = -1 * self.popt[0] * (r - r_value)  # param 2

        rospy.loginfo_throttle(0.1, ['r_dot' + str(r_dot)])

        x_dot = r_dot * math.cos(theta_circle) - r * theta_circle_dot * math.sin(theta_circle)
        y_dot = r_dot * math.sin(theta_circle) + r * theta_circle_dot * math.cos(theta_circle)

        xd = math.cos(theta) * (1.0 / a) * x_dot - math.sin(theta) * (1.0 / b) * y_dot
        yd = math.sin(theta) * (1.0 / a) * x_dot + math.cos(theta) * (1.0 / b) * y_dot

        v_new = np.array([xd, yd, -2 * x[2], 0, 0, 0])
        # v=0.9*self.v_old+0.1*v_new
        v = v_new
        self.v_old = v

        return v

    def init_params(self):
        # initialize params for kinova
        self.num_joints = 6
        self.ee_real_logged = False
        self.robot_desired_joints = Base_JointSpeeds()
        self.robot_desired = TwistCommand()

    def signal_handler(self, signal, frame):
        print("\n Program exiting gracefully")
        # stop robot
        vel_cmd = [0, 0, 0, 0, 0, 0]
        self.publish_robot_desired(vel_cmd)
        sys.exit(0)

    def get_robot_pos(self):
        try:
            self.trans_ee_real = self.listener.lookupTransform('/base_link', '/end_effector_link', rospy.Time(0))
            if not self.ee_real_logged:
                self.ee_real_logged = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        self.end_effector_position = np.array(self.trans_ee_real[0])
        self.end_efector_orientation = np.array(self.trans_ee_real[1])  # A quaternion

    def init_ros(self):
        rospy.init_node('Mocap', anonymous=True)
        self.rate = rospy.Rate(FREQ)
        self.robot_pub = rospy.Publisher("/my_gen3/in/cartesian_velocity", TwistCommand, queue_size=3)
        self.red_cup = rospy.Subscriber("/Red_cup/pose", PoseStamped, self.tf_redcup)
        self.listener = tf.TransformListener()

    def set_pose_down(self, position):
        self.target_position = position
        self.desired_orientation = np.array([math.pi -0.1, math.pi/2, 0])  # Robot end effector facing down
        self.cmd_received = False
        self.ee_real_logged = False
        self.gain = 1

    def calculate_robot_desired_attractor(self, ):
        vel_cmd = np.zeros(6)
        vel_cmd[:3] = (self.target_position - self.end_effector_position) * self.gain
        vel_cmd[3:6] = self.desired_orientation
        return vel_cmd

    def calculate_robot_desired_orientation(self, ):
        vel_cmd = np.zeros(6)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.end_efector_orientation)
        vel_cmd[3:6] = (self.desired_orientation - [roll, yaw, pitch]) * self.gain
        return vel_cmd

    def quat_to_direction(self, quat):
        R0 = tf.transformations.quaternion_matrix(quat)
        angle, vec, _ = tf.transformations.rotation_from_matrix(R0)
        return angle * vec

    def quat_to_euler(self, quat):
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quat)
        euler = np.zeros(3)
        euler[0] = roll
        euler[1] = yaw
        euler[2] = pitch
        return euler

    def publish_robot_desired(self, vel_cmd):
        msg = Twist()
        msg.linear_x = vel_cmd[0]
        msg.linear_y = vel_cmd[1]
        msg.linear_z = vel_cmd[2]
        msg.angular_x = vel_cmd[3]
        msg.angular_y = vel_cmd[4]
        msg.angular_z = vel_cmd[5]
        self.robot_desired.twist = msg
        self.robot_desired.reference_frame = 0
        self.robot_desired.duration = 0
        self.robot_pub.publish(self.robot_desired)

    def set_pose(self, position):
        self.target_position = position
        self.desired_orientation = np.array([0, 0, 0])  # Robot end effector facing down
        self.cmd_received = False
        self.ee_real_logged = False
        self.gain = 1

    def tf_redcup(self, data):
        global last_cup
        global active_cup
        global once_1

        if once_1:
            last_cup = data
            self.initial_cup = data
            once_1 = False
        if data.pose != last_cup.pose:
            last_cup = data
            active_cup = True
            # rospy.loginfo("Red cup at \n%s", data.pose.position)
            self.cup = data
        elif data.pose == last_cup.pose:
            active_cup = False


if __name__ == "__main__":
    Mocap()
