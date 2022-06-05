<!DOCTYPE html>
<html>
<head>
	<meta charset = "UTF-8">
	<meta http-equiv = "refresh" content = "30">
	<style type = "text/css">
		.spec{
			text-align:center;
		}
		.con{
			text-align:left;
		}
	</style>
</head>
<body>
	<h1 align = "center">My Database</h1>
	<div class = "spec">
		# <b>The sensor value desciption</b><br>
    	# 0 ~ 255 RED <br>
    	# 0 ~ 255 GREEN <br>
    	# 0 ~ 255 BLUE <br>
    	# 0 ~ 255 CLEAR
	</div>
	<table border = '1' style = "width = 30%" align = "center">
		<tr align = "center">
	    	<th>ID</th>
      		<th>DATE</th>
	    	<th>TIME</th>
      		<th>RED</th>
      		<th>GREEN</th>
      		<th>BLUE</th>
      		<th>CLEAR</th>
		</tr>
		<?php
	    	$conn = mysqli_connect("localhost","root","kcci");
      		mysqli_select_db($conn, "project");
      		$result = mysqli_query($conn, "select * from color");

		    while($row = mysqli_fetch_array($result)){
		    echo "<tr align = center>";
    		echo '<td>'.$row['ID'].'</td>';
		    echo '<td>'.$row['DATE'].'</td>';
		    echo '<td>'.$row['TIME'].'</td>';
		    echo '<td>'.$row['RED'].'</td>';
		    echo '<td>'.$row['GREEN'].'</td>';
		    echo '<td>'.$row['BLUE'].'</td>';
		    echo '<td>'.$row['CLEAR'].'</td>';
		    echo "</tr>";
		    mysqli_close($conn);
      		}
		?>
	</table>
</body>
</html>
