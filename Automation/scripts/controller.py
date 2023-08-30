import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt

from robotics_hackathon_automation.msg import PointArray 
from geometry_msgs.msg import Point

from collections import deque # array of array - pointers array kinda


import numpy as np

### LOOK at commit ID: 25f0e6aec96f4ab1a437ab29428f13c16f8eb8b0
# for perfect robot working



x = -5.06+1.79
y = -3.12+0.66
theta = 0.0


# List of points to visit
# points = [
#     [(-5.06, -3.12), (-5.088418491884296, -3.035145357926098), (-5.112365173201178, -2.918068598434703), (-5.1272912236507375, -2.663878783208241), (-5.0632862582198515, -2.3811981852437425), (-5.047389163139106, -1.957727714886989), (-5.064057379641183, -1.5588124696679229), (-4.859541880326859, -1.2513497426230136), (-4.819389714604883, -1.2913236747856882), (-4.730206976687626, -1.3015577062731976), (-4.701967324035544, -1.3373260319035023), (-4.671530485017849, -1.3906051804477462), (-4.657150660814364, -1.3833915678023345), (-4.585774690154912, -1.375661547099686), (-4.51584913576658, -1.361443018372245), (-4.509376426309283, -1.3189928757613916), (-4.464163273789211, -1.3113915602345845), (-4.459817784278702, -1.3022596027832114), (-4.4459821958607595, -1.3029293461877836), (-4.442390510774949, -1.2979731071793097), (-4.418549683176905, -1.307669495674152), (-4.355031281423723, -1.3216208363557524), (-4.320288967331228, -1.3831989178161062), (-4.3125785732234245, -1.441974287985153), (-4.261580831574404, -1.4699384285664028), (-4.201763758886193, -1.5144138986766347), (-4.216565223861122, -1.5698511793317724), (-4.234880231238499, -1.6045113133816644), (-4.233523964664006, -1.6579665176994762), (-4.177136689059108, -1.683521733541641), (-4.143396891798913, -1.7323315459021), (-4.098268940466883, -1.7660060081926483), (-3.61, -2.2)]
# ,
# [(-3.61, -2.2), (-3.6767545023195414, -1.8024230466744373), (-3.741171743234541, -1.5362757417447392), (-3.850301652634379, -1.5201904200602936), (-3.9508025725370493, -1.531669891173751), (-4.020325159791937, -1.4983176369563185), (-4.079938094356813, -1.4969734687213248), (-4.117297013095577, -1.4667674698796174), (-4.228449074693901, -1.504720184995467), (-4.330403560853815, -1.4039977230108946), (-4.4359902032174885, -1.3541336483419868), (-4.443226280512722, -1.3448281454120452), (-4.537504777933091, -1.3317500384548304), (-4.554880623771647, -1.3007884694145557), (-4.63488382376559, -1.3055339100882934), (-4.671860285331986, -1.3386664924058604), (-4.721674034011145, -1.3448071017386594), (-4.769934907458539, -1.3457158201447885), (-4.804039209105303, -1.3144887523357174), (-4.843935875050863, -1.2760424371130226), (-4.8941045024910625, -1.0458595639273924), (-4.9390357211096045, -0.9758453177672701), (-4.898483260698509, -0.6842074482825169), (-4.919950502284122, -0.35861210751935496), (-4.77865031168592, -0.04430911280816591), (-4.682546072960437, 0.02880439184787595), (-4.665269954165951, 0.14206174489779377), (-4.707922268295653, 0.2418608841546851), (-4.720275766975034, 0.4683288795867466), (-4.405749287952847, 0.6703744890266401), (-4.223970758059971, 0.8019902542140303), (-4.163971820495714, 0.8586071933433095), (-3.8993493699837596, 0.8362714217838044), (-3.7472135464275858, 0.8756290758233968), (-3.413569674231688, 0.9316037675638849), (-3.060716381084207, 1.0226471965287949), (-3.0782384676678114, 1.0220631403879732), (-3.085772804876332, 1.0996961715819717), (-3.0929978610116264, 1.1911086601596022), (-3.146221506244464, 1.3691582943091598), (-3.065404461216023, 1.4837094677095748), (-2.9771883264631707, 1.6250129665246351), (-2.982659460999897, 1.7607048908246687), (-2.4473768700296152, 1.789919647454712), (-2.3350857170195933, 1.7941380612280402), (-1.9656282674070842, 1.8050204941318106), (-2.28, 1.86)]
# ,
# [(-2.28, 1.86), (-2.3222759332125302, 1.843883455475647), (-2.4211741645169957, 1.7575047875812917), (-2.5500926542394713, 1.6774725228561458), (-2.5736742777729336, 1.649647464087616), (-2.7130967082635546, 1.6406895488837971), (-2.77590002784058, 1.7542423960708644), (-2.896784134977214, 1.7974431166338147), (-3.091758568904559, 1.716595405394562), (-3.30071123452892, 1.8153237873739743), (-3.4156233555831994, 1.6642415077762196), (-3.5079673829729128, 1.3169037223485858), (-3.4496850947072906, 0.9927098071966658), (-3.4417277885211996, 0.9450902992332946), (-3.4739515176022104, 0.7398849803555536), (-3.6145087866735452, 0.6512224705993135), (-3.728099064762164, 0.5679630127729651), (-3.8409150699097254, 0.5974704837976179), (-3.9262217571957656, 0.5720601220421114), (-3.9852216142921466, 0.5853708405225985), (-4.07239458011161, 0.5363182291965249), (-4.166061919468205, 0.5974936262196414), (-4.165771330997117, 0.6025942975773906), (-4.252356204485753, 0.6085148579899611), (-4.287669961871956, 0.546381128302643), (-4.327944110496595, 0.5717165900609905), (-4.403163266099901, 0.6086079736202419), (-4.470984337359067, 0.5827584221415645), (-4.558527841009429, 0.5312659697252096), (-4.630080742909002, 0.478671207786549), (-4.694547402410909, 0.3587261011084982), (-4.700700097444294, 0.16330095735147215), (-4.682381213945851, 0.12507383031893188), (-4.733090180767821, -0.039666620693707244), (-4.71935391007196, -0.08495659545721601), (-4.572006134709537, -0.27091114282162965), (-4.5531843522200806, -0.42130646059690446), (-4.45974678828301, -0.45540591130320407), (-4.401610068774356, -0.4831695391286675), (-4.325794733867816, -0.48616482937166894), (-4.284671772979371, -0.4424687037793693), (-4.223699411397413, -0.41485385400324504), (-4.044281479930346, -0.5244062785861403), (-4.035949777410928, -0.5116990899995422), (-4.0083292540371565, -0.4781470820084307), (-3.933873515823418, -0.5371156008269828), (-3.9314788339918088, -0.5421090652626777), (-3.8605195453266243, -0.5450242746675987), (-3.8122841977506363, -0.5150176532645463), (-3.7749806396719103, -0.5293635392330744), (-3.6832489785543525, -0.536775800438288), (-3.567676854002781, -0.5433974023425351), (-3.5199185232430334, -0.5044935995380958), (-3.4401990691103226, -0.5289502960880846), (-3.386917055163832, -0.49574991846472644), (-3.3102491719629885, -0.49866375095773946), (-2.8421537751363752, -0.6579314489591843), (-2.50200038659436, -0.677789249445808), (-2.5081802077804873, -0.7691571504861152), (-2.519938817702228, -0.7819147458800791), (-2.5549633962995473, -0.900159469153011), (-2.649420857204573, -1.034275033100294), (-2.5896483129103793, -1.151767271301527), (-2.601155888035998, -1.2190386163303475), (-2.5305439661752, -1.2734182114078436), (-2.553544222864933, -1.3525108133938804), (-2.4690076496725424, -1.4174402024641763), (-2.4574155473901724, -1.4560305828319038), (-2.363781700490933, -1.5182123466637067), (-2.2819565930918615, -1.5254035301575875), (-2.1103758376535784, -1.5716990636893218), (-1.682557331505503, -1.619211705859098), (-1.1559743482711329, -1.6420204969176317), (-0.8847017117632852, -1.4765808270644227), (-0.8681558244113449, -1.3921999787826977), (-0.8278180160144972, -1.2596606032906281), (-0.8230331515682967, -1.2162140224386708), (-0.8689337683689373, -1.198885809892196), (-0.8808004176129214, -1.1903185148993272), (-0.8356010119981622, -1.0108418381735869), (-0.8164610007496156, -0.8780811903097865), (-0.8089665099869238, -0.7640564941842741), (-0.8047697894027626, -0.6689560723961633), (-0.8152272517475074, -0.5384922492592408), (-0.83067429144328, -0.40809990843882177), (-0.8469891339608134, -0.3371141660131639), (-0.8647269722923626, -0.2222557160245417), (-0.8062071438775678, -0.12210630203795679), (-0.8047312996559622, -0.047234964704988086), (-0.8329071025058741, 0.024922725199618703), (-0.8665307278213954, 0.08653582384520335), (-0.8712898542087151, 0.19756933880269506), (-0.7666294144176206, 0.25648787031953263), (-0.178869284855536, 0.3379424683667829), (0.09319218187907558, 0.2088501841280366), (0.57, 0.33)]
# ,
# [(0.57, 0.33), (0.5224091697116909, 0.3672778229547146), (0.3704628653968015, 0.5178524272340135), (0.12521452698310773, 0.5400377567350665), (0.10224486384401033, 0.4787862835893313), (0.015299234842864162, 0.32211930777182973), (-0.5172308371312628, 0.2944552560173455), (-0.9471940687006901, 0.3948248535831844), (-1.1184581022003448, 0.5047090963145467), (-1.204327463125709, 0.6127092410428847), (-1.212437020049472, 0.6404127304006407), (-1.191836733526037, 0.6815080081611289), (-1.1850682329687354, 0.7668270219785864), (-1.176707615413858, 0.9007731103295633), (-1.1258256865664393, 1.0862934136751758), (-1.174801540158039, 1.2640467813724667), (-1.194606490092959, 1.3762238483355507), (-1.153451165600735, 1.4721168027929568), (-1.1465373755569483, 1.4914283088997144), (-1.0286343464068235, 1.6226233976268287), (-1.0228425897230373, 1.745437474239047), (-0.918620319416613, 1.7968170066114957), (-0.8999710298498997, 1.8238162201859973), (-0.8887457951385386, 1.831720759004711), (-0.8185049264358318, 1.8296831901799573), (-0.7887251561641402, 1.9288890265931642), (-0.8013403702699907, 1.9825357283024168), (-0.8083202177175157, 2.030640714046422), (-0.7352445081351789, 2.0449972013689144), (-0.7040444052438803, 2.1234975735166213), (-0.5956452907089858, 2.1263765318270575), (-0.5378625239556327, 2.093717838701811), (-0.4625527044551703, 2.100895805771956), (-0.3536261001825976, 2.1278183692244355), (-0.32607507953217857, 2.07472965872846), (-0.2666728579513364, 2.051134962931309), (-0.19397473873447083, 2.11716923320534), (-0.12182418656721217, 2.1011745081940107), (-0.11420846765092542, 2.059957389467869), (0.0730097397753479, 2.0832802962412855), (0.12568986131376006, 2.0695547480708285), (0.22484211492469433, 2.0488180570998855), (0.5929040745439651, 2.0848014106760893), (0.7668328027351698, 2.090237450223889), (0.8667220622176954, 2.0264214774782383), (1.3604734189716927, 1.9922521709390193), (1.3660150237961843, 1.9622757922991847), (1.3423882540143328, 1.9088691696333795), (1.3491994388629547, 1.8768413484059774), (1.317634678695242, 1.8472998439063815), (1.3608524467484449, 1.7390863988676373), (1.4462385946528766, 1.625213920371048), (1.681504460372285, 1.4741240408952931), (1.7164618926802462, 1.3782420892278853), (1.6929445871005468, 1.3073626526415003), (1.7218478757262798, 1.2929682554279038), (1.697157396427703, 1.2284497870099926), (1.709023118689231, 1.1887332423235215), (1.6994402969126268, 1.1587250829521323), (1.6911915237723822, 1.1040635137526205), (1.6826543439434798, 1.0961701589580117), (1.6453988881068167, 1.089082214150138), (1.6355563066166414, 1.027986889031784), (1.6852998372782315, 0.9157394582583616), (1.7121904862101636, 0.8979951397324896), (1.693334733993742, 0.8146940582524594), (1.6788801829416187, 0.7934493903084997), (1.6673377396873668, 0.7573900842350823), (1.6681304848908955, 0.7059964691596108), (1.661701443842474, 0.6401235542770275), (1.703185069918491, 0.5000552014093412), (1.7034019665301636, 0.4238070706211971), (1.8913443150835332, 0.15800429327316973), (2.1154230079779657, 0.15557635071429873), (2.204352803112485, 0.1131785383842013), (2.190688724426289, 0.05852540721351194), (2.193883916256157, -0.06354400223188114), (2.2106846153053894, -0.07270183441881121), (2.216217880022933, -0.14986483548760549), (2.1924728626653036, -0.2773859966363218), (2.1753717879412546, -0.2726144976310082), (2.118885083734958, -0.4012690046749079), (2.112339222299626, -0.5215393864180125), (2.1410648475986434, -0.5777108266544112), (2.0624353850492683, -0.7211475950327237), (2.046661634555728, -0.7778902721287337), (2.071082845478933, -0.8035415196528849), (2.1230824565471345, -0.922472024814464), (2.101239873082063, -0.9742478359038332), (2.022172239162306, -1.0093637197660466), (1.9845992204436906, -1.0105613750634013), (1.9075839627953501, -1.0071554573344237), (1.895541893901773, -1.0043496637872007), (1.8287426218956648, -1.0109979874921697), (1.8031439058139136, -1.011444102000931), (1.7413325432826205, -1.006019675918198), (1.7240780068896944, -0.9864106333094427), (1.7206692046614418, -0.9894871959738875), (1.681562435192204, -0.9684774299433072), (1.643922876162638, -1.0215127040987646), (1.5616365116627329, -1.0729343138026026), (1.471966659770023, -1.1673402443921541), (1.376643571541379, -1.2406239205475014), (1.3527845829586262, -1.2494534664715398), (1.2755910147540508, -1.3045147593650972), (1.298990159598881, -1.374014529998505), (1.284960652930979, -1.420678581239276), (1.268460544518744, -1.4617710845041032), (1.2612929190371152, -1.471012815847238), (1.277574127164303, -1.5126007043354701), (1.2588827405421992, -1.524360309305892), (1.2615710982956971, -1.5649630145928728), (1.2879904849150134, -1.636459017550691), (1.58, -2.26)]
# ,
# [(1.58, -2.26), (1.369042914330488, -2.0554556755775635), (1.44487452380266, -1.974463750297057), (1.28753911728065, -1.576995528944638), (1.3900913517300708, -1.1217739350019802), (1.7213297913321346, -1.0052338431522414), (1.8207842294111956, -0.6146827627487434), (1.8612466252343145, -0.6080173090719295), (1.8605239469988573, -0.6127478900905519), (1.862469095582489, -0.6292489363957453), (1.9029363225184006, -0.6622781280965846), (1.9111755395340952, -0.6833444165017389), (1.9498691824024708, -0.7122980587877199), (2.005537476353885, -0.7589382393788422), (2.122379495315238, -0.8595720485501835), (2.1327773890944113, -0.8532378930323858), (2.2260341615623513, -0.8900799901629937), (2.228993963780184, -0.9622178143444429), (2.2196608117475036, -0.978567302945961), (2.2191785918061853, -1.0419253414370597), (2.2281092200207704, -1.0768247561130257), (2.269522379777209, -1.1403504923377126), (2.3474210246145137, -1.217208197849548), (2.3977812635370723, -1.2948740187174392), (2.39341146860512, -1.3405769913903574), (2.4765648183706204, -1.4192250717533326), (2.5636943862346766, -1.4170975456738588), (2.5877229543225337, -1.4637803102762272), (2.6764750886342803, -1.5212582264707408), (2.7079639630687393, -1.5360173400331725), (2.7293824146230543, -1.5180179931382487), (2.7851285229731615, -1.5287968265847212), (2.918379982779385, -1.5516733111556), (2.964421262963398, -1.5799560112404594), (3.1549959649825103, -1.6468792107481411), (3.1628817388163832, -1.6898629051901346), (3.396042518902723, -1.6841912072478793), (3.436790747636226, -1.713092107777362), (3.4901397039136755, -1.7446995564392824), (3.679789611233092, -1.7467325992608442), (3.7651729198316715, -1.7390037370028104), (3.8753747848702904, -1.7213967435657525), (4.0866370757276265, -1.7066527370881215), (4.160907070568622, -1.6924959691448638), (4.168988791986927, -1.7117864992722656), (4.207093099190114, -1.765056000082262), (4.285821090739435, -1.809821413098915), (4.320708578134983, -1.8462682747547863), (4.314858386363145, -1.8577568211398303), (4.310507095296567, -1.8996279485164103), (4.298197655614918, -1.932324224035403), (4.301138226351329, -1.9545347298090172), (4.3421408163542035, -1.973410776753837), (4.3360995921649925, -1.9950896667915754), (4.329686076409822, -2.008020374659713), (4.313626175279016, -2.020384708640597), (4.322296567263015, -2.0456087318906007), (4.34013226024643, -2.0527914662437197), (4.356087933622728, -2.0735870040009514), (4.471631037006461, -2.0957577726066257), (4.578202651578082, -2.102873011464281), (5.18, -2.19)] 
# ]


goals = [(-3.61, -2.2), (-2.28, 1.86), (0.57, 0.33), (1.58, -2.26), (5.18, -2.19)]

goals = [(round(x+1.79, 2), round(y+0.66, 2)) for x, y in goals]



# for w in points:
#     points[w] = [(round(x+1.79, 2), round(y+0.66, 2)) for x, y in points[w]]


# points = [[(round(x+1.79, 2), round(y+0.66, 2)) for x, y in sublist] for sublist in points]


# print(points)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    # theta = msg.pose.pose.orientation.w
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("tb3_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

points = [[(0.0, 0.0), (1.0, 1.0)],]
can_go = False

goal = Point()

def point_array_callback(msg):
        global points
        global can_go
        global i 
        global j
        x = msg.pointss
        print(x)
        modified_points = [(point.x + 1.79, point.y + 0.66) for point in x]
        print(modified_points)
        points.extend(modified_points)
        # points.append(msg.pointss)
        print("helloooo")
        # points = [[(round(x+1.79, 2), round(y+0.66, 2)) for x, y in sublist] for sublist in points]
        
        
        
        print(points)
        if(not can_go):
            
            i = 0 #points in the tragectory
            j = 0 # paths

            goal.x = points[j][i][0]
            goal.y = points[j][i][1]
            print(goal)
        can_go = True
    # if current_point_array is None:
    #     process_next_point_array()


rospy.Subscriber('/planned_path', PointArray, point_array_callback)




r = rospy.Rate(100)

# goal.x = -5.0308+1.79
# goal.y = -2.96+0.66


def dist_raw(a,b):
    euclid_dist = sqrt(a**2 + b**2)
    return euclid_dist

while (not rospy.is_shutdown()):

    if(can_go):
        inc_x = goal.x -x
        inc_y = goal.y -y

        angle_to_goal = atan2(inc_y, inc_x)
        
        if(dist_raw(inc_x, inc_y)<0.05 and (angle_to_goal-theta) < 0.1):
            i= i+1
            goal.x = points[j][i][0]
            goal.y = points[j][i][1]
            # print(goal)

        calculate_d_goal = dist_raw((goals[j][0]- x),goals[j][1]- y)
        # print(calculate_d_goal)
        if( calculate_d_goal < 0.4):
            i = len(points[j])
            print("reached")
            speed.linear.x = 0
            speed.angular.z = 0
            j = j + 1
            i = 1
        else:
            if (angle_to_goal - theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 1
            elif (angle_to_goal - theta) < -0.1:
                speed.linear.x = 0.0
                speed.angular.z = -1
            else:
                speed.linear.x = 0.2
                speed.angular.z = 0.0
        
            

    pub.publish(speed)
    r.sleep()

