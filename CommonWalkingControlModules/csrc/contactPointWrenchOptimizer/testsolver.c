/* Produced by CVXGEN, 2013-01-22 10:35:59 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 10000
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.A[0] = 0.20319161029830202;
  params.A[1] = 0.8325912904724193;
  params.A[2] = -0.8363810443482227;
  params.A[3] = 0.04331042079065206;
  params.A[4] = 1.5717878173906188;
  params.A[5] = 1.5851723557337523;
  params.A[6] = -1.497658758144655;
  params.A[7] = -1.171028487447253;
  params.A[8] = -1.7941311867966805;
  params.A[9] = -0.23676062539745413;
  params.A[10] = -1.8804951564857322;
  params.A[11] = -0.17266710242115568;
  params.A[12] = 0.596576190459043;
  params.A[13] = -0.8860508694080989;
  params.A[14] = 0.7050196079205251;
  params.A[15] = 0.3634512696654033;
  params.A[16] = -1.9040724704913385;
  params.A[17] = 0.23541635196352795;
  params.A[18] = -0.9629902123701384;
  params.A[19] = -0.3395952119597214;
  params.A[20] = -0.865899672914725;
  params.A[21] = 0.7725516732519853;
  params.A[22] = -0.23818512931704205;
  params.A[23] = -1.372529046100147;
  params.A[24] = 0.17859607212737894;
  params.A[25] = 1.1212590580454682;
  params.A[26] = -0.774545870495281;
  params.A[27] = -1.1121684642712744;
  params.A[28] = -0.44811496977740495;
  params.A[29] = 1.7455345994417217;
  params.A[30] = 1.9039816898917352;
  params.A[31] = 0.6895347036512547;
  params.A[32] = 1.6113364341535923;
  params.A[33] = 1.383003485172717;
  params.A[34] = -0.48802383468444344;
  params.A[35] = -1.631131964513103;
  params.A[36] = 0.6136436100941447;
  params.A[37] = 0.2313630495538037;
  params.A[38] = -0.5537409477496875;
  params.A[39] = -1.0997819806406723;
  params.A[40] = -0.3739203344950055;
  params.A[41] = -0.12423900520332376;
  params.A[42] = -0.923057686995755;
  params.A[43] = -0.8328289030982696;
  params.A[44] = -0.16925440270808823;
  params.A[45] = 1.442135651787706;
  params.A[46] = 0.34501161787128565;
  params.A[47] = -0.8660485502711608;
  params.A[48] = -0.8880899735055947;
  params.A[49] = -0.1815116979122129;
  params.A[50] = -1.17835862158005;
  params.A[51] = -1.1944851558277074;
  params.A[52] = 0.05614023926976763;
  params.A[53] = -1.6510825248767813;
  params.A[54] = -0.06565787059365391;
  params.A[55] = -0.5512951504486665;
  params.A[56] = 0.8307464872626844;
  params.A[57] = 0.9869848924080182;
  params.A[58] = 0.7643716874230573;
  params.A[59] = 0.7567216550196565;
  params.A[60] = -0.5055995034042868;
  params.A[61] = 0.6725392189410702;
  params.A[62] = -0.6406053441727284;
  params.A[63] = 0.29117547947550015;
  params.A[64] = -0.6967713677405021;
  params.A[65] = -0.21941980294587182;
  params.A[66] = -1.753884276680243;
  params.A[67] = -1.0292983112626475;
  params.A[68] = 1.8864104246942706;
  params.A[69] = -1.077663182579704;
  params.A[70] = 0.7659100437893209;
  params.A[71] = 0.6019074328549583;
  params.A[72] = 0.8957565577499285;
  params.A[73] = -0.09964555746227477;
  params.A[74] = 0.38665509840745127;
  params.A[75] = -1.7321223042686946;
  params.A[76] = -1.7097514487110663;
  params.A[77] = -1.2040958948116867;
  params.A[78] = -1.3925560119658358;
  params.A[79] = -1.5995826216742213;
  params.A[80] = -1.4828245415645833;
  params.A[81] = 0.21311092723061398;
  params.A[82] = -1.248740700304487;
  params.A[83] = 1.808404972124833;
  params.A[84] = 0.7264471152297065;
  params.A[85] = 0.16407869343908477;
  params.A[86] = 0.8287224032315907;
  params.A[87] = -0.9444533161899464;
  params.A[88] = 1.7069027370149112;
  params.A[89] = 1.3567722311998827;
  params.A[90] = 0.9052779937121489;
  params.A[91] = -0.07904017565835986;
  params.A[92] = 1.3684127435065871;
  params.A[93] = 0.979009293697437;
  params.A[94] = 0.6413036255984501;
  params.A[95] = 1.6559010680237511;
  params.A[96] = 0.5346622551502991;
  params.A[97] = -0.5362376605895625;
  params.A[98] = 0.2113782926017822;
  params.A[99] = -1.2144776931994525;
  params.A[100] = -1.2317108144255875;
  params.A[101] = 0.9026784957312834;
  params.A[102] = 1.1397468137245244;
  params.A[103] = 1.8883934547350631;
  params.A[104] = 1.4038856681660068;
  params.A[105] = 0.17437730638329096;
  params.A[106] = -1.6408365219077408;
  params.A[107] = -0.04450702153554875;
  params.A[108] = 1.7117453902485025;
  params.A[109] = 1.1504727980139053;
  params.A[110] = -0.05962309578364744;
  params.A[111] = -0.1788825540764547;
  params.A[112] = -1.1280569263625857;
  params.A[113] = -1.2911464767927057;
  params.A[114] = -1.7055053231225696;
  params.A[115] = 1.56957275034837;
  params.A[116] = 0.5607064675962357;
  params.A[117] = -1.4266707301147146;
  params.A[118] = -0.3434923211351708;
  params.A[119] = -1.8035643024085055;
  params.A[120] = -1.1625066019105454;
  params.A[121] = 0.9228324965161532;
  params.A[122] = 0.6044910817663975;
  params.A[123] = -0.0840868104920891;
  params.A[124] = -0.900877978017443;
  params.A[125] = 0.608892500264739;
  params.A[126] = 1.8257980452695217;
  params.A[127] = -0.25791777529922877;
  params.A[128] = -1.7194699796493191;
  params.A[129] = -1.7690740487081298;
  params.A[130] = -1.6685159248097703;
  params.A[131] = 1.8388287490128845;
  params.A[132] = 0.16304334474597537;
  params.A[133] = 1.3498497306788897;
  params.A[134] = -1.3198658230514613;
  params.A[135] = -0.9586197090843394;
  params.A[136] = 0.7679100474913709;
  params.A[137] = 1.5822813125679343;
  params.A[138] = -0.6372460621593619;
  params.A[139] = -1.741307208038867;
  params.A[140] = 1.456478677642575;
  params.A[141] = -0.8365102166820959;
  params.A[142] = 0.9643296255982503;
  params.A[143] = -1.367865381194024;
  params.A[144] = 0.7798537405635035;
  params.A[145] = 1.3656784761245926;
  params.A[146] = 0.9086083149868371;
  params.A[147] = -0.5635699005460344;
  params.A[148] = 0.9067590059607915;
  params.A[149] = -1.4421315032701587;
  params.A[150] = -0.7447235390671119;
  params.A[151] = -0.32166897326822186;
  params.A[152] = 1.5088481557772684;
  params.A[153] = -1.385039165715428;
  params.A[154] = 1.5204991609972622;
  params.A[155] = 1.1958572768832156;
  params.A[156] = 1.8864971883119228;
  params.A[157] = -0.5291880667861584;
  params.A[158] = -1.1802409243688836;
  params.A[159] = -1.037718718661604;
  params.A[160] = 1.3114512056856835;
  params.A[161] = 1.8609125943756615;
  params.A[162] = 0.7952399935216938;
  params.A[163] = -0.07001183290468038;
  params.A[164] = -0.8518009412754686;
  params.A[165] = 1.3347515373726386;
  params.A[166] = 1.4887180335977037;
  params.A[167] = -1.6314736327976336;
  params.A[168] = -1.1362021159208933;
  params.A[169] = 1.327044361831466;
  params.A[170] = 1.3932155883179842;
  params.A[171] = -0.7413880049440107;
  params.A[172] = -0.8828216126125747;
  params.A[173] = -0.27673991192616;
  params.A[174] = 0.15778600105866714;
  params.A[175] = -1.6177327399735457;
  params.A[176] = 1.3476485548544606;
  params.A[177] = 0.13893948140528378;
  params.A[178] = 1.0998712601636944;
  params.A[179] = -1.0766549376946926;
  params.A[180] = 1.8611734044254629;
  params.A[181] = 1.0041092292735172;
  params.A[182] = -0.6276245424321543;
  params.A[183] = 1.794110587839819;
  params.A[184] = 0.8020471158650913;
  params.A[185] = 1.362244341944948;
  params.A[186] = -1.8180107765765245;
  params.A[187] = -1.7774338357932473;
  params.A[188] = 0.9709490941985153;
  params.A[189] = -0.7812542682064318;
  params.A[190] = 0.0671374633729811;
  params.A[191] = -1.374950305314906;
  params.W[0] = 1.9118096386279388;
  params.W[1] = 0.011004190697677885;
  params.W[2] = 1.3160043138989015;
  params.W[3] = -1.7038488148800144;
  params.W[4] = -0.08433819112864738;
  params.W[5] = -1.7508820783768964;
  params.C[0] = 1.8842414310877373;
  params.C[1] = 1.445810178712959;
  params.C[2] = 1.0685499182618368;
  params.C[3] = 1.076496282315957;
  params.C[4] = 1.53879265800317;
  params.C[5] = 1.0755664045052309;
  params.epsilon[0] = 0.3675446360248855;
  params.B[0] = -0.2545716633339441;
  params.B[1] = -0.008868675926170244;
  params.B[2] = 0.3332476609670296;
  params.B[3] = 0.48205072561962936;
  params.B[4] = -0.5087540014293261;
  params.B[5] = 0.4749463319223195;
  params.B[6] = -1.371021366459455;
  params.B[7] = -0.8979660982652256;
  params.B[8] = 1.194873082385242;
  params.B[9] = -1.3876427970939353;
  params.B[10] = -1.106708108457053;
  params.B[11] = -1.0280872812241797;
  params.B[12] = -0.08197078070773234;
  params.B[13] = -1.9970179118324083;
  params.B[14] = -1.878754557910134;
  params.B[15] = -0.15380739340877803;
  params.B[16] = -1.349917260533923;
  params.B[17] = 0.7180072150931407;
  params.B[18] = 1.1808183487065538;
  params.B[19] = 0.31265343495084075;
  params.B[20] = 0.7790599086928229;
  params.B[21] = -0.4361679370644853;
  params.B[22] = -1.8148151880282066;
  params.B[23] = -0.24231386948140266;
  params.B[24] = -0.5120787511622411;
  params.B[25] = 0.3880129688013203;
  params.B[26] = -1.4631273212038676;
  params.B[27] = -1.0891484131126563;
  params.B[28] = 1.2591296661091191;
  params.B[29] = -0.9426978934391474;
  params.B[30] = -0.358719180371347;
  params.B[31] = 1.7438887059831263;
  params.B[32] = -0.8977901479165817;
  params.B[33] = -1.4188401645857445;
  params.B[34] = 0.8080805173258092;
  params.B[35] = 0.2682662017650985;
  params.B[36] = 0.44637534218638786;
  params.B[37] = -1.8318765960257055;
  params.B[38] = -0.3309324209710929;
  params.B[39] = -1.9829342633313622;
  params.B[40] = -1.013858124556442;
  params.B[41] = 0.8242247343360254;
  params.B[42] = -1.753837136317201;
  params.B[43] = -0.8212260055868805;
  params.B[44] = 1.9524510112487126;
  params.B[45] = 1.884888920907902;
  params.B[46] = -0.0726144452811801;
  params.B[47] = 0.9427735461129836;
  params.B[48] = 0.5306230967445558;
  params.B[49] = -0.1372277142250531;
  params.B[50] = 1.4282657305652786;
  params.B[51] = -1.309926991335284;
  params.B[52] = 1.3137276889764422;
  params.B[53] = -1.8317219061667278;
  params.B[54] = 1.4678147672511939;
  params.B[55] = 0.703986349872991;
  params.B[56] = -0.2163435603565258;
  params.B[57] = 0.6862809905371079;
  params.B[58] = -0.15852598444303245;
  params.B[59] = 1.1200128895143409;
  params.B[60] = -1.5462236645435308;
  params.B[61] = 0.0326297153944215;
  params.B[62] = 1.4859581597754916;
  params.B[63] = 1.71011710324809;
  params.fmin[0] = -1.1186546738067493;
  params.fmin[1] = -0.9922787897815244;
}
