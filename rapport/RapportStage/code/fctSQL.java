/**
 * @description Permet de créer une requête d'insertion dans la base de données
 * @param name : nom du registre
 * @param value : valeur du registre
 * @throws SQLException
 */
public void insert(String name, Object value)throws SQLException{
	String sql = "INSERT sensorValue (registerName, registerValue) VALUES " + "('" + name + "','" + value +"')";
    statement.execute(sql);
}


/**
 * @description Permet de lire une partie ou toute la base de données
 * @param select : '*' pour tout ou seulement une colonne => [currDateTime], [registerName], [registerValue]
 * @param options : peut être ORDER BY, WHERE, GROUP BY...
 * @throws SQLException
 */
public void select(String select, String options)throws SQLException{
	String sql = "SELECT " + select + " FROM sensorValue " + options;
	try {
	    rs = statement.executeQuery(sql);
	    while(rs.next()) {
	    	System.out.println(rs.getTimestamp("currDateTime")+ " " + rs.getString("registerName")+ " " + rs.getFloat("registerValue"));
	    }
	} catch(SQLException sq) {}
}