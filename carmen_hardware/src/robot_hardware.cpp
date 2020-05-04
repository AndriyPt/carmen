
bool RobotHardware::init(node)
{
   bool result = false;
   ReadSettingsCommand cmd;

   cmd.p = node.getIntParam("p", 0);
   cmd.i = node.getIntParam("i", 0);
   cmd.d = node.getIntParam("d", 0);

   try
   {
       ReadSettingsResult result = OrionMasterHelper.SendAndReceive<ReadSettingsCommand, ReadSettingsResult>(
            cmd, 0.1, 2);

       node.setIntParam("p", result.p);
       node.setIntParam("i", result.i);
       node.setIntParam("d", result.d);
       result = true;
   }
   catch (CommunicationException ex)
   {
     ROS_ERROR("Communication Exception %d %s", ex.GetCode(), ex.GetMessage());
   }
   return result;
}

void RobotHardware::read()
{
}

void RobotHardware::write()
{
}
