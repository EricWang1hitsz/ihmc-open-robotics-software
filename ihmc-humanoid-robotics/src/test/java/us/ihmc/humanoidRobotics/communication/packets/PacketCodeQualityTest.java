package us.ihmc.humanoidRobotics.communication.packets;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import org.junit.Test;
import org.reflections.Reflections;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

@ContinuousIntegrationPlan(categories = IntegrationCategory.HEALTH)
public class PacketCodeQualityTest
{
   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = 30000)
   public void testOnlyEmptyConstructor()
   {
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithoutEmptyConstructor = new HashSet<>();
      Set<Class<? extends Packet>> packetTypesWithNonEmptyConstructors = new HashSet<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            List<Constructor<?>> constructors = Arrays.asList(packetType.getConstructors());
            assertFalse("The type: " + packetType.getSimpleName() + " has no constructors?!", constructors.isEmpty());

            boolean hasEmptyConstructor = constructors.stream().filter(constructor -> constructor.getParameterTypes().length == 0).findFirst().isPresent();
            if (!hasEmptyConstructor)
               packetTypesWithoutEmptyConstructor.add(packetType);

            boolean hasNonEmptyConstructors = constructors.stream().filter(constructor -> constructor.getParameterTypes().length != 0).findFirst().isPresent();
            if (hasNonEmptyConstructors)
               packetTypesWithNonEmptyConstructors.add(packetType);

            packetTypesWithNonEmptyConstructors.add(packetType);
         }
         catch (SecurityException e)
         {
         }
      }

      if (verbose)
      {
         if (!packetTypesWithoutEmptyConstructor.isEmpty())
         {
            PrintTools.error("List of packet sub-types without an empty constructor:");
            packetTypesWithoutEmptyConstructor.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
         if (!packetTypesWithNonEmptyConstructors.isEmpty())
         {
            PrintTools.error("List of packet sub-types with non-empty constructors:");
            packetTypesWithNonEmptyConstructors.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertFalse("Packet sub-types should implement an empty constructor.", packetTypesWithoutEmptyConstructor.isEmpty());
      assertTrue("Packet sub-types should not implement a non-empty constructor.", packetTypesWithNonEmptyConstructors.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 1.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = 30000)
   public void testNoRandomConstructor()
   {
      boolean printPacketTypesWithRandomConstructor = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithRandomConstructor = new HashSet<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            packetType.getConstructor(Random.class);
            // If we get here, that means the type implement a random constructor.
            if (printPacketTypesWithRandomConstructor)
               PrintTools.error("Found type that implements a random constructor: " + packetType.getSimpleName());
            packetTypesWithRandomConstructor.add(packetType);
         }
         catch (NoSuchMethodException | SecurityException e)
         {
         }
      }

      assertTrue("Packet sub-types should not implement a random constructor.", packetTypesWithRandomConstructor.isEmpty());
   }


	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAllPacketFieldsArePublic()
   {
      IHMCCommunicationKryoNetClassList classList = new IHMCCommunicationKryoNetClassList();

      for (Class<?> clazz : classList.getPacketClassList())
      {
         checkIfAllFieldsArePublic(clazz);
      }

      for (Class<?> clazz : classList.getPacketFieldList())
      {
         checkIfAllFieldsArePublic(clazz);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAllRosExportedPacketsHaveRandomConstructor()
   {
      IHMCCommunicationKryoNetClassList classList = new IHMCCommunicationKryoNetClassList();
      Set<Class> badClasses = new HashSet<>();

      for (Class<?> clazz : classList.getPacketClassList())
      {
         checkIfClassHasRandomConstructor(clazz, badClasses);
      }

      if(!badClasses.isEmpty())
      {
         System.err.println("PacketCodeQualityTest.checkIfClassHasRandomConstructor failed: The following classes do not have Random constructors:");
         for (Class badClass : badClasses)
         {
            System.err.println("- " + badClass.getCanonicalName());
         }

         fail("PacketCodeQualityTest.checkIfClassHasRandomConstructor failed. Consult Standard Error logs for list of classes without Random constructors.");
      }
   }

   private void checkIfClassHasRandomConstructor(Class<?> clazz, Set<Class> badClasses)
   {
      // Skip base class
      if(clazz == Packet.class)
      {
         return;
      }

      if(Packet.class.isAssignableFrom(clazz) && !Modifier.isAbstract(clazz.getModifiers()) && clazz.isAnnotationPresent(RosMessagePacket.class))
      {
         try
         {
            Constructor<?> constructor = clazz.getConstructor(Random.class);
         }
         catch (NoSuchMethodException e)
         {
            badClasses.add(clazz);
         }
      }
   }

   private void checkIfAllFieldsArePublic(Class<?> clazz)
   {
      if (clazz == String.class) return;
      if (clazz == ArrayList.class) return;

      for (Field field : clazz.getDeclaredFields())
      {
         if (Modifier.isStatic(field.getModifiers())) continue;
         if (Modifier.isTransient(field.getModifiers())) continue;
         if (field.isSynthetic()) continue;

         assertTrue("Class " + clazz.getCanonicalName() + " has non-public field " + field.getName() + " declared by " + field.getDeclaringClass().getCanonicalName(), Modifier.isPublic(field.getModifiers()));
         assertFalse("Class " + clazz.getCanonicalName() + " has final field " + field.getName() + " declared by " + field.getDeclaringClass().getCanonicalName(), Modifier.isFinal(field.getModifiers()));
      }
   }
}
