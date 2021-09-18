/**
 * @description Permet de lire les variables de type %MW sur l'automate
 * @throws SQLException
 */
public static void readAddrW() throws SQLException {
	try {
		InputRegister[] inReg = master.readInputRegisters(addrReg, sizeReg);
		if(inReg == null) {
			System.out.println("Problème de lecture");
			System.exit(1);
		}
		for(int i = 0; i < inReg.length; i++) {
		System.out.print(inReg[i] + " ");
		}
		System.out.println();
		
		requete.insert("MW"+ addrReg, inReg[0]);
		
	} catch (ModbusException e) {
		e.printStackTrace();
		System.exit(1);
	}
}

/**
 * @description Permet de lire les variables de type %M sur l'automate
 * @throws SQLException
 */
public static void readAddrM() throws SQLException {
	try {
		BitVector inReg = master.readInputDiscretes(addrReg, sizeReg);
		if(inReg == null) {
			System.out.println("Problème de lecture");
			System.exit(1);
		}
		byte[] tab = inReg.getBytes();
		System.out.println(tab[0]);
		
		requete.insert("M"+ addrReg, tab[0]);
		
	} catch (ModbusException e) {
		e.printStackTrace();
		System.exit(1);
	}
}