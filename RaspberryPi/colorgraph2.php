<?php
	$conn = mysqli_connect("localhost", "root", "kcci");
	mysqli_set_charset($conn, "UTF8");
	mysqli_select_db($conn, "project");
	$result=mysqli_query($conn, "select DATE, TIME, CLEAR from color");
	$data = array(array('project', 'C'));
	if($result){
		while($row=mysqli_fetch_array($result)){
	  		array_push($data, array($row[0]."\n".$row[1], intval($row['CLEAR'])));
	  	}
	}
	$options=array(
		'title' => 'clear light', 'width' => 1000, 'height' => 500,
		'colors' => ['#ffff00']
	);
?>

<script src="//www.google.com/jsapi"></script>
<script>
	let data = <?= json_encode($data) ?>;
	let options = <?= json_encode($options) ?>;
	google.load('visualization', '1.0', {'packages':['corechart']});
	google.setOnLoadCallback(function() {
		let chart = new google.visualization.ColumnChart(document.querySelector('#chart_div'));
    	chart.draw(google.visualization.arrayToDataTable(data), options);
    });
</script>
<div id="chart_div"></div>
