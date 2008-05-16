<html>
  <head>
  <title>Aircraft Web Interface</title>
<!-- ----------------------------------------------------------------------- -->
<!-- This script updates the DSM time displays by fetching the time from     -->
<!-- the XML based printStatus messages being multicast to the server.       -->
<!-- ----------------------------------------------------------------------- -->
    <script type='text/javascript' src='jsolait/init.js'></script>
    <script type='text/javascript' src='periodic.js'></script>
    <link href='index.css' rel='stylesheet' type='text/css'>
  </head>

<!-- ----------------------------------------------------------------------- -->
<!-- Detect if this page is loaded across a slow connection (i.e. satcom).   -->
<!-- ----------------------------------------------------------------------- -->
<script>
var is_periodic=(document.location.hostname == "localhost" ||
                 document.location.hostname == "acserver" ||
                 document.location.hostname == "acserver.raf.ucar.edu" ||
                 document.location.hostname == "hyper.guest.ucar.edu");

if (is_periodic == false)
  document.title = document.title + " (static page)";
</script>

<?php
include_once('utils/utils.php');

//-- ----------------------------------------------------------------------- -->
//-- Query the dsm_server for a list of DSM names and locations.             -->
//-- ----------------------------------------------------------------------- -->
$dsmList = xu_rpc_http_concise( array( 'method' => 'GetDsmList',
                                       'args'   => '',
                                       'host'   => 'localhost',
                                       'uri'    => '/RPC2',
                                       'port'   => '30003',
                                       'debug'  =>  '0',
                                       'output' => 'xmlrpc' ));

// TODO - see if xu_rpc_http_concise provides any status variables to test here instead.
if ($dsmList == "")
  exit("<h5>DSM server not responding!</h5>");
?>


<script>
<!-- ----------------------------------------------------------------------- -->
<!-- The Hidden iframe below will contain javascript that will call recvResp.-->
<!-- ----------------------------------------------------------------------- -->
var xmlrpc = importModule("xmlrpc");
GetClocks = new xmlrpc.XMLRPCMethod('xmlrpc.php?port=30006&method=GetClocks', '');

GetStatus = new Array();
<?php foreach ($dsmList as $key => $val) { ?>
GetStatus['<?=$key?>'] =
  new xmlrpc.XMLRPCMethod('xmlrpc.php?port=30006&method=GetStatus&args=<?=$key?>', '');
<?php } ?>
GetStatus['dsm_server'] =
  new xmlrpc.XMLRPCMethod('xmlrpc.php?port=30006&method=GetStatus&args=dsm_server', '');
GetStatus['nimbus'] =
  new xmlrpc.XMLRPCMethod('xmlrpc.php?port=30006&method=GetStatus&args=nimbus', '');

var selectedDsm = '';
function recvResp(content) {
  document.getElementById('stat').innerHTML = content;
}
function selectDsm(content) {
  if (is_periodic == false)
    periodic.UpdateClocks();
  selectedDsm = content;
  if (selectedDsm)
    recvResp( GetStatus[selectedDsm]() );
  else
    recvResp( "" );
}
function clicker(that) {
  if (that.selectedIndex+1 > 0)
    return that.options[that.selectedIndex].id;
  else
    return '';
}
</script>


<!-- ----------------------------------------------------------------------- -->
<!-- Table layout:  +--------------------------+--------------------------+  -->
<!--                |                          |                          |  -->
<!--                | control                  | image                    |  -->
<!--                |                          |                          |  -->
<!--                +--------------------------+--------------------------+  -->
<!--                |                                                     |  -->
<!--                | status / response                                   |  -->
<!--                |                                                     |  -->
<!--                +-----------------------------------------------------+  -->
<!-- ----------------------------------------------------------------------- -->
<table border><tbody align='center' valign='top'>
<tr>
<td>
<!--
<center>flight number: rf02</center>
<br>
-->

<!-- ----------------------------------------------------------------------- -->
<!-- Procvide help for static pages.                                         -->
<!-- ----------------------------------------------------------------------- -->
<p id="paraID"> </p>
<script>
if (is_periodic == false) {
  mypara = document.getElementById("paraID");
  mytext = document.createTextNode("to refresh click on an item:");
  mypara.appendChild(mytext); 
}
</script>

<!-- ----------------------------------------------------------------------- -->
<!-- This form provides a selection of DSMs to control.  There are two       -->
<!-- steps in this form: the selection of the dsm and the choice of action.  -->
<!--                                                                         -->
<!-- TODO - higlight the option red when in warning...                       -->
<!--  <option value='YYY'                        >YYY (----/--/-- --:--:--)  -->
<!--  <option value='XXX' style='color: #ff0000;'>YYY (----/--/-- --:--:--)  -->
<!-- ----------------------------------------------------------------------- -->

<form action='control_dsm.php' method='POST' target='scriptframe'>

  <select name='dsm[]' size="12" multiple="multiple"
         onclick='selectDsm(clicker(this))'>
    <?php foreach ($dsmList as $key => $val) { ?>
    <option value='<?=$key?>' id='<?=$key?>' label='<?=str_pad($val,12, '_')?>'>
       <?=str_pad($val,12, '_')?> (---- -- -- --:--:--)</option>
    <?php } ?>
    <option value='dsm_server' id='dsm_server' label='dsm_server__'>
       dsm_server__ (---- -- -- --:--:--)</option>
    <option value='nimbus'     id='nimbus'     label='nimbus______'>
       nimbus______ (---- -- -- --:--:--)</option>
  </select><p>

  action:&nbsp
  <select name='act'>
    <option value='' selected>
    <option>Start</option>
    <option>Stop</option>
    <option>Restart</option>
    <option>Quit</option>
<!--     <option>calibrate analog</option> -->
  </select>

  &nbsp
  <input type=submit value='submit &raquo;' class='button'
         onclick="selectedDsm=''; recvResp('working...')">

</form>
</td>
<td>
<img src='GV-top-228x201.jpg'>
</td>
</tr>
<tr>
<!-- ----------------------------------------------------------------------- -->
<!-- The status table is displayed in this row.  The Hidden iframe is used   -->
<!-- to receive javascript commands that refreash this row as well.          -->
<!-- ----------------------------------------------------------------------- -->
<iframe name='scriptframe' width=0 height=0 frameborder=0 src=''</iframe>
<td id='stat' colspan=2></td>
</tr>
</tbody></table>
</html>
