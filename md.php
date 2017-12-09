<!doctype html>
<?php

$api_url = "https://api.particle.io/v1/devices/";
$device_id = "REPLACE_ME";
$access_token = "REPLACE_ME";

function moveDoor() {
    global $api_url, $device_id, $access_token;
    $url = $api_url.$device_id."/door1move?access_token=".$access_token;
    $ch = curl_init($url);
    curl_setopt( $ch, CURLOPT_POST, true);
    curl_setopt( $ch, CURLOPT_RETURNTRANSFER, true);
    curl_setopt( $ch, CURLOPT_POSTFIELDS, array('args' => "1"));
    $data = curl_exec($ch);
    curl_close($ch);
    //print($data);
}

function getDoorState() {
    global $api_url, $device_id, $access_token;
    $url = $api_url.$device_id."/doorstate?access_token=".$access_token;
    $ch = curl_init($url);
    curl_setopt( $ch, CURLOPT_GET, true);
    curl_setopt( $ch, CURLOPT_RETURNTRANSFER, true);
    #curl_setopt( $ch, CURLOPT_POSTFIELDS, array('args' => "1"));
    $data = curl_exec($ch);
    curl_close($ch);
    $obj = json_decode($data);
    #print "AARARAR".$obj->result;
    return $obj->result;
}

$method = $_SERVER['REQUEST_METHOD'];
if ($method == 'GET') {
    if ($_GET['move'] == 1) {
        moveDoor();
    }
}

?>
<html lang="en">
    <head>
        <meta charset="utf-8">
        <title>GD move</title>
    </head>
    <body>
        <div>Door is: <?php echo getDoorState() ?></div>
        <div>
            <form action="" method="get">
                <button type="submit" name="move" value="1">Door</button>
            </form>
        </div>
    </body>
</html>
