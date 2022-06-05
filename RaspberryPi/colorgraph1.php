<?php
	$conn = mysqli_connect("localhost", "root", "kcci");
	mysqli_set_charset($conn, "UTF8");
	mysqli_select_db($conn, "project");
	$result=mysqli_query($conn, "select DATE, TIME, RED, GREEN, BLUE from color");
	$data = array(array('project', 'R', 'G', 'B'));
	if($result){
		while($row=mysqli_fetch_array($result)){
	  		array_push($data, array($row[0]."\n".$row[1], intval($row[2]), intval($row[3]), intval($row[4])));
	  	}
	}
	$options=array(
		'title' => 'RGB', 'width' => 1000, 'height' => 500,
		'colors' => ['#ff0000', '#00ff00', '#0000ff']
	);
?>

<script src="//www.google.com/jsapi"></script>
<script>
	let data = <?= json_encode($data) ?>;
	let options = <?= json_encode($options) ?>;
	google.load('visualization', '1.0', {'packages':['corechart']});
	google.setOnLoadCallback(function() {
		let chart = new google.visualization.LineChart(document.querySelector('#chart_div'));
    	chart.draw(google.visualization.arrayToDataTable(data), options);
    });
</script>
<div id="chart_div"></div>
