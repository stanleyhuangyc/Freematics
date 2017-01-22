<?php 
  $server_key = "TEST_SERVER_KEY"; // replace it with your server key and keep it secret
  $server_url = "http://hub.freematics.com/".$server_key.$_GET["api"]."?";
  
  if (!$_GET["api"]) {
    echo "No API specified";
    exit;
  }

  // pass through all request arguments
  foreach ($_GET as $key => $value) { 
    if ($key !== "api") {
      $server_url .= $key."=".$value."&";
    }
  } 
  
  // invoke API
  $ch = curl_init();
  curl_setopt($ch, CURLOPT_URL, $server_url);
  curl_setopt($ch, CURLOPT_RETURNTRANSFER, true);
  $result = curl_exec($ch);
  curl_close($ch);

  // forward response
  header("Content-Type: application/json");
  header("Cache-Control: no-store, no-cache, must-revalidate, max-age=0");
  header("Cache-Control: post-check=0, pre-check=0", false);
  header("Pragma: no-cache");
  echo $result;
?>