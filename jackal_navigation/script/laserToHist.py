import matplotlib.pyplot as plt

ranges= [3.8022944927215576, 3.7782418727874756, 3.7557690143585205, 3.73024845123291, 3.7079572677612305, 3.684950113296509, 3.6622705459594727, 3.6429851055145264, 3.62087082862854, 3.6004087924957275, 3.578887462615967, 3.5569968223571777, 3.540957450866699, 3.5216920375823975, 3.5020394325256348, 3.481813907623291, 3.4644150733947754, 3.446068048477173, 3.4288506507873535, 3.4113962650299072, 3.394582509994507, 3.3781344890594482, 3.363835096359253, 3.345294237136841, 3.330310821533203, 3.3132522106170654, 3.3003692626953125, 3.282588243484497, 3.2685039043426514, 3.293401002883911, 3.3281073570251465, 3.292968273162842, 3.194282293319702, 3.1025028228759766, 3.0168371200561523, 2.9345457553863525, 2.8567745685577393, 2.78324294090271, 2.712604284286499, 2.6461329460144043, 2.5859451293945312, 2.5246384143829346, 2.4655344486236572, 2.4125304222106934, 2.3609931468963623, 2.310852289199829, 2.2634027004241943, 2.217578887939453, 2.1743247509002686, 2.132812023162842, 2.091320753097534, 2.05401611328125, 2.0150437355041504, 1.9806747436523438, 1.9455227851867676, 1.9116771221160889, 1.8818830251693726, 1.851552963256836, 1.8193974494934082, 1.7914985418319702, 1.7630736827850342, 1.7376834154129028, 1.710328459739685, 1.688024640083313, 1.6829320192337036, 1.636400818824768, 1.615721583366394, 1.5932992696762085, 1.572977900505066, 1.5513596534729004, 1.5278267860412598, 1.5102612972259521, 1.4906843900680542, 1.4743664264678955, 1.4547077417373657, 1.437235951423645, 1.4197701215744019, 1.4050869941711426, 1.3867260217666626, 1.3711903095245361, 1.355986475944519, 1.3390908241271973, 1.3274203538894653, 1.3124786615371704, 1.2975972890853882, 1.284684181213379, 1.2717156410217285, 1.260251760482788, 1.2468969821929932, 1.233811855316162, 1.2223384380340576, 1.210615873336792, 1.1991339921951294, 1.1895546913146973, 1.179455041885376, 1.1669232845306396, 1.1580010652542114, 1.1461968421936035, 1.1376217603683472, 1.1295908689498901, 1.1173622608184814, 1.109954595565796, 1.0991865396499634, 1.089270830154419, 1.0793745517730713, 1.0729944705963135, 1.0638056993484497, 1.0549339056015015, 1.0486695766448975, 1.0406049489974976, 1.034528374671936, 1.0263975858688354, 1.0192248821258545, 1.0118850469589233, 1.002947211265564, 0.9970809817314148, 0.9894022941589355, 0.9824796319007874, 0.9763714671134949, 0.9711967706680298, 0.9619381427764893, 0.9593230485916138, 0.9512872099876404, 0.9457889795303345, 0.9401472806930542, 0.9376539587974548, 0.929347813129425, 0.9214343428611755, 0.9159783720970154, 0.9123289585113525, 0.9092994332313538, 0.9018237590789795, 0.8984472751617432, 0.8919093012809753, 0.8881090879440308, 0.8832169771194458, 0.8789591193199158, 0.8755959868431091, 0.8692508339881897, 0.8644598722457886, 0.8619869351387024, 0.8574578762054443, 0.8525393605232239, 0.8481504917144775, 0.8466293811798096, 0.8401705622673035, 0.836219310760498, 0.8326597213745117, 0.829792320728302, 0.8257497549057007, 0.8224738836288452, 0.8186965584754944, 0.8149922490119934, 0.8117430806159973, 0.8083363175392151, 0.805686891078949, 0.800965428352356, 0.799227774143219, 0.7958336472511292, 0.7938430905342102, 0.791334867477417, 0.7860356569290161, 0.7837594151496887, 0.7805798053741455, 0.7799957990646362, 0.7752223610877991, 0.7722041606903076, 0.7699207067489624, 0.7685713768005371, 0.7665930986404419, 0.7625672817230225, 0.7609419226646423, 0.758515477180481, 0.7544662952423096, 0.7555338740348816, 0.7526057362556458, 0.7497789263725281, 0.7477851510047913, 0.7449208498001099, 0.7430078387260437, 0.7412471771240234, 0.7395875453948975, 0.7383803725242615, 0.7367132306098938, 0.7326037883758545, 0.7323564291000366, 0.7317157983779907, 0.7294905185699463, 0.7254916429519653, 0.7252665162086487, 0.7248558402061462, 0.7203347682952881, 0.7209888696670532, 0.7193669676780701, 0.7185448408126831, 0.7178752422332764, 0.7143480181694031, 0.7138910293579102, 0.713032066822052, 0.7087880373001099, 0.7103039622306824, 0.7084770798683167, 0.7066808938980103, 0.7069063186645508, 0.7047995924949646, 0.7046987414360046, 0.7026015520095825, 0.7020117044448853, 0.7019513249397278, 0.7009586095809937, 0.6993788480758667, 0.6976380348205566, 0.6977932453155518, 0.6975444555282593, 0.6977887153625488, 0.6965254545211792, 0.6945466995239258, 0.6929713487625122, 0.6929078698158264, 0.6925534605979919, 0.6902751326560974, 0.6936001181602478, 0.6918667554855347, 0.6901006698608398, 0.690970778465271, 0.6914290189743042, 0.6898221969604492, 0.688481867313385, 0.6885172724723816, 0.6882566809654236, 0.6885034441947937, 0.6887543797492981, 0.6873261332511902, 0.6884821653366089, 0.6854656338691711, 0.687689483165741, 0.6868909001350403, 0.6885981559753418, 0.6900559663772583, 0.6887004971504211, 0.6867768168449402, 0.6880580186843872, 0.6870505213737488, 0.6884742975234985, 0.688332200050354, 0.6872355341911316, 0.688146710395813, 0.6865439414978027, 0.6891136765480042, 0.6883291006088257, 0.6884218454360962, 0.6883547306060791, 0.6904499530792236, 0.6882067918777466, 0.6923075914382935, 0.6926642656326294, 0.6915587186813354, 0.6925265789031982, 0.6935186982154846, 0.6934065222740173, 0.6947517991065979, 0.694989025592804, 0.695097804069519, 0.6954624652862549, 0.6970934867858887, 0.6982779502868652, 0.69706130027771, 0.7001303434371948, 0.7019798159599304, 0.7027127742767334, 0.7036816477775574, 0.7032797336578369, 0.7031704783439636, 0.7058730721473694, 0.7055689096450806, 0.7075533270835876, 0.7090057730674744, 0.7106533646583557, 0.7112318277359009, 0.711058497428894, 0.712654709815979, 0.7159408330917358, 0.7157173752784729, 0.718677818775177, 0.7195104956626892, 0.7209759950637817, 0.7201731204986572, 0.723482608795166, 0.7238158583641052, 0.7983448505401611, 0.7848744988441467, 0.7672938704490662, 0.7552784085273743, 0.7423075437545776, 0.7375397086143494, 0.7365268468856812, 0.738922119140625, 0.741850733757019, 0.7436317801475525, 0.7461186647415161, 0.7461806535720825, 0.7503029704093933, 0.7504244446754456, 0.754962146282196, 0.7550410032272339, 0.7561428546905518, 0.7598444819450378, 0.7626612186431885, 0.7645564079284668, 0.7673721313476562, 0.7714449167251587, 0.771253764629364, 0.7751303911209106, 0.779005229473114, 0.7812297344207764, 0.783639669418335, 0.7881880402565002, 0.7889686822891235, 0.7927961349487305, 0.7949369549751282, 0.7988771200180054, 0.8004725575447083, 0.8059958815574646, 0.8087066411972046, 0.8115624189376831, 0.8147316575050354, 0.8180897235870361, 0.820927083492279, 0.8251439929008484, 0.8304699063301086, 0.8325271010398865, 0.8350874185562134, 0.8411619067192078, 0.845364511013031, 0.8493640422821045, 0.8510432839393616, 0.8569843769073486, 0.8613199591636658, 0.8652074337005615, 0.8699889183044434, 0.8718934655189514, 0.878254771232605, 0.8832532167434692, 0.8873942494392395, 0.8904822468757629, 0.8960878849029541, 0.9003624320030212, 0.9069825410842896, 0.913454532623291, 0.9179949164390564, 0.922345757484436, 0.9289610981941223, 0.9340292811393738, 0.9409385919570923, 0.9430742263793945, 0.9505760669708252, 0.9539234638214111, 0.9618461728096008, 0.9709073305130005, 0.9776424765586853, 0.9834698438644409, 0.9892639517784119, 0.9964078664779663, 1.0041404962539673, 1.0094610452651978, 1.018912672996521, 1.0237765312194824, 1.0305397510528564, 1.041124701499939, 1.0455979108810425, 1.0542155504226685, 1.065045952796936, 1.069441318511963, 1.079148530960083, 1.0889739990234375, 1.0984039306640625, 1.1071853637695312, 1.115252137184143, 1.124513864517212, 1.1358059644699097, 1.1445683240890503, 1.1548187732696533, 1.167298674583435, 1.1766639947891235, 1.1857742071151733, 1.1973204612731934, 1.2093318700790405, 1.2205674648284912, 1.2331936359405518, 1.2447458505630493, 1.2575379610061646, 1.2706968784332275, 1.281510829925537, 1.297529697418213, 1.3109718561172485, 1.3262414932250977, 1.338725209236145, 1.3538427352905273, 1.3676592111587524, 1.385371446609497, 1.4017447233200073, 1.4169739484786987, 1.434012532234192, 1.4515419006347656, 1.468196988105774, 1.4887057542800903, 1.5089689493179321, 1.52567720413208, 1.5466821193695068, 1.5687602758407593, 1.5893219709396362, 1.611667513847351, 1.634774923324585, 1.6586472988128662, 1.6806353330612183, 1.7083625793457031, 1.7316550016403198, 1.7593343257904053, 1.7886937856674194, 1.8152776956558228, 1.8457962274551392, 1.8738399744033813, 1.9075812101364136, 1.938231110572815, 1.9729340076446533, 2.0099449157714844, 2.0479629039764404, 2.082192897796631, 2.1374166011810303, 2.1643311977386475, 2.2086405754089355, 2.2564797401428223, 2.302401542663574, 2.3501453399658203, 2.4017109870910645, 2.455083131790161, 2.5123136043548584, 2.5708513259887695, 2.634033441543579, 2.6989004611968994, 2.7690114974975586, 2.8431100845336914, 2.917341947555542, 2.9984824657440186, 3.0850484371185303, 3.1760001182556152, 3.2741153240203857, 3.377295970916748, 3.4873769283294678, 3.602936029434204, 3.732379198074341, 3.867931604385376, 4.013670921325684, 4.173079967498779, 4.343240261077881, 4.528862476348877, 4.730651378631592, 4.950580596923828, 5.192288398742676, 5.4597625732421875, 5.728500843048096, 5.747755527496338, 5.774158000946045, 5.813357830047607, 5.878419399261475, 10.568232536315918, 10.562050819396973, 10.557331085205078, 10.550909042358398, 10.543610572814941, 10.540786743164062, 10.537160873413086, 10.53433895111084, 10.529569625854492, 10.52691650390625, 10.525243759155273, 10.52206802368164, 10.520018577575684, 10.522109031677246, 10.52143383026123, 10.520069122314453, 10.521430015563965, 10.521997451782227, 10.522454261779785, 10.522720336914062, 10.525887489318848, 10.528495788574219, 10.531171798706055, 10.535332679748535, 10.537919044494629, 10.545833587646484, 10.547786712646484, 10.551668167114258, 10.558891296386719, 10.565252304077148, 10.57254695892334, 10.580052375793457, 10.584933280944824, 10.596768379211426, 10.602071762084961, 8.21312141418457, 8.132424354553223, 8.087373733520508, 7.626621246337891, 7.332015037536621, 7.061988830566406, 6.810279369354248, 6.5749831199646, 6.358190059661865, 6.1773786544799805, 5.963653087615967, 5.785516738891602, 5.615475654602051, 5.457120895385742, 5.3073601722717285, 5.164334774017334, 5.032031536102295, 4.903099060058594, 4.78249454498291, 4.668878078460693, 4.5606184005737305, 4.4629011154174805, 4.354969024658203, 4.259313583374023, 4.169653415679932, 4.083090305328369, 3.999626636505127, 3.921229600906372, 3.844698190689087, 3.7715282440185547, 3.701781988143921, 3.633244514465332, 3.5679268836975098, 3.5076522827148438, 3.446945905685425, 3.388686180114746, 3.3321356773376465, 3.2791171073913574, 3.226196765899658, 3.174954414367676, 3.1276490688323975, 3.0812737941741943, 3.0340535640716553, 2.9922268390655518, 2.948624849319458, 2.9078853130340576, 2.866641044616699, 2.8276193141937256, 2.81056547164917, 2.7531380653381348, 2.7182319164276123, 2.6827900409698486, 2.648451089859009, 2.617161273956299, 2.5848610401153564, 2.5539355278015137, 2.525299072265625, 2.497274160385132, 2.467665195465088, 2.439168691635132, 2.4132394790649414, 2.385054588317871, 2.3611247539520264, 2.3361356258392334, 2.311110496520996, 2.2872841358184814, 2.2649452686309814, 2.2423839569091797, 2.2193195819854736, 2.197396755218506, 2.1783294677734375, 2.1545474529266357, 2.135744094848633, 2.1163408756256104, 2.097388505935669, 2.0784225463867188, 2.0617446899414062, 2.0447213649749756, 2.025693416595459, 2.007244825363159, 1.9935001134872437, 1.9737985134124756, 1.958260416984558, 1.9413785934448242, 1.9284546375274658, 1.9113425016403198, 1.8998451232910156, 1.8838316202163696, 1.8699904680252075, 1.8559041023254395, 1.840531587600708, 1.8289086818695068, 1.813502311706543, 1.8044689893722534, 1.7915655374526978, 1.7783249616622925, 1.7672165632247925, 1.7546638250350952, 1.7438499927520752, 1.7306439876556396, 1.720215916633606, 1.708210825920105, 1.7003686428070068, 1.6894687414169312, 1.678589105606079, 1.6690737009048462, 1.658801555633545, 1.6490631103515625, 1.6393744945526123, 1.6295405626296997, 1.6205201148986816, 1.6098566055297852, 1.6032313108444214, 1.59426748752594, 1.585964560508728, 1.5756474733352661, 1.5700757503509521, 1.561127781867981, 1.5538111925125122, 1.5426404476165771, 1.5361931324005127, 1.5308454036712646, 1.523260235786438, 1.5145900249481201, 1.5086441040039062, 1.5003883838653564, 1.4936989545822144, 1.487827181816101, 1.4818944931030273, 1.4738339185714722, 1.4670535326004028, 1.4644275903701782, 1.4562925100326538, 1.4516545534133911, 1.450865387916565, 1.4670230150222778, 1.4826185703277588, 1.4267690181732178, 1.4217077493667603, 1.4163662195205688, 1.4109246730804443, 1.4080158472061157, 1.4014062881469727, 1.3955552577972412, 1.3911634683609009, 1.3890019655227661, 1.3814146518707275, 1.379239559173584, 1.3717010021209717, 1.3694791793823242, 1.3642793893814087, 1.361484408378601, 1.3563928604125977, 1.3506821393966675, 1.3487941026687622, 1.3456522226333618, 1.33950936794281, 1.3370883464813232, 1.3314603567123413, 1.3301352262496948, 1.3248193264007568, 1.3219400644302368, 1.3191083669662476, 1.314811110496521, 1.3126705884933472, 1.3099995851516724, 1.3055860996246338, 1.302383542060852, 1.3004831075668335, 1.298237919807434, 1.294593334197998, 1.2910332679748535, 1.2898210287094116, 1.2870659828186035, 1.2853883504867554, 1.281543254852295, 1.278716802597046, 1.2772120237350464, 1.2748817205429077, 1.273996114730835, 1.2704771757125854, 1.2684530019760132, 1.266162395477295, 1.2612212896347046, 1.2635669708251953, 1.2618478536605835, 1.2595393657684326, 1.2567378282546997, 1.253869652748108, 1.2555001974105835, 1.2531728744506836, 1.250587821006775, 1.2503715753555298, 1.248358964920044, 1.247391700744629, 1.2465571165084839, 1.2434661388397217, 1.2432656288146973, 1.2408039569854736, 1.2418760061264038, 1.2399508953094482, 1.2386404275894165, 1.2386229038238525, 1.2365529537200928, 1.2365071773529053, 1.2356467247009277, 1.2337356805801392, 1.2353241443634033, 1.236488938331604, 1.2349367141723633, 1.2321741580963135, 1.2325100898742676, 1.2329576015472412, 1.233994960784912, 1.2326226234436035, 1.230814814567566, 1.2316899299621582, 1.231124997138977, 1.2307111024856567, 1.2306206226348877]

_ = plt.hist(ranges)

_ = plt.xlabel('number of raser hits')
_ = plt.ylabel('distance')

plt.show()
