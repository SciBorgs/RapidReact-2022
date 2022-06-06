package frc.robot.autoProfile;

public enum TransportProfile {
    // 0/1-ball (shoot)
    R_0(),
    
    // 2-ball (shoot-move-shoot)
    R_1B("path1B"),
    R_1L("path1L"),
    R_2B("path2B"),
    R_2D("path2D"),
    R_3D("path3D"),
    R_4E("path4E"),
    R_4G("path4G"),

    // 3-ball (shoot-move-shoot-shoot)
    R_1BL("path1BL"),
    R_3DE("path3DE"),
    R_4ED("path4ED"),
    R_4EG("path4EG"),

    // 3/4-ball (shoot-move-shoot-move-shoot(-shoot))
    R_3DM("path3D", "pathDM"),
    R_4EM("path4E", "pathEM"),

    // 4/5-ball (shoot-move-shoot-shoot-move-shoot(-shoot))
    R_4EDM("path4ED", "pathDM"),
    
    // Test
    TEST("Test-Peanut", "Test-Circle");

    protected final int transportStages;
    private final String[] pathnames;

    private TransportProfile(String... pathnames) {
        this.transportStages = pathnames.length;
        this.pathnames = pathnames;
    }

    public String getStagePath(int i) {
        if (i > transportStages) throw new IllegalArgumentException();
        return pathnames[i - 1]; // starts at 1
    }
}
